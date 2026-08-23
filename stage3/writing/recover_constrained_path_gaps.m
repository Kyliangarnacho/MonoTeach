function recovery = recover_constrained_path_gaps(robotContext, recoveryWindows, ...
        postureConfig, recoveryConfig, positionOnlySeedLookup)
%RECOVER_CONSTRAINED_PATH_GAPS Recover internal Writing gaps by aiming relaxation.
%
% This is a derived, robot-specific artifact.  It preserves every original
% XYZ target and every strict Writing failure while retrying only the failed
% targets with a shared, deterministic tool-direction tolerance ladder.

    validate_inputs(robotContext, recoveryWindows, postureConfig, ...
        recoveryConfig, positionOnlySeedLookup);
    model = robotContext.model;
    endEffector = char(string(robotContext.end_effector));
    limits = robotContext.joint_limits;
    gik = configured_gik(model, recoveryConfig.solver_algorithm, ...
        recoveryConfig.allow_random_restart);
    windowResults = repmat(empty_window_result(), 1, numel(recoveryWindows));

    for windowIndex = 1:numel(recoveryWindows)
        windowResults(windowIndex) = recover_one_window( ...
            recoveryWindows(windowIndex), model, gik, endEffector, limits, ...
            postureConfig, recoveryConfig, positionOnlySeedLookup);
    end

    recovery = struct();
    recovery.artifact_type = 'ConstrainedPathGapRecovery';
    recovery.robot_context_id = robotContext.id;
    recovery.task_definition = 'fixed_xyz_plus_relaxed_body_local_z_aiming';
    recovery.recovery_config = recoveryConfig;
    recovery.recovery_windows = recoveryWindows;
    recovery.window_results = windowResults;
    recovery.summary = summarize_recovery(windowResults);
end


function result = recover_one_window(window, model, gik, endEffector, limits, ...
        postureConfig, config, positionLookup)

    validate_window(window, size(limits, 1));
    result = empty_window_result();
    result.gap_index = window.gap_index;
    result.preik_segment_index = window.preik_segment_index;
    result.original_run_index = window.original_run_index;
    result.left_success_index = window.left_success_index;
    result.right_success_index = window.right_success_index;
    result.failed_indices = window.failed_indices;
    result.strict_failure_reasons = window.strict_failure_reasons;
    result.left_accepted_q = window.left_accepted_q;
    result.right_accepted_q = window.right_accepted_q;
    result.original_writing_targets = window.original_writing_targets;

    previousQ = window.left_accepted_q;
    recovered = repmat(empty_recovered_point(), 1, 0);
    unrecovered = repmat(empty_unrecovered_point(), 1, 0);
    for targetIndex = 1:numel(window.original_writing_targets)
        target = window.original_writing_targets(targetIndex);
        initialProvenance = 'left_success_writing_q';
        if ~isempty(recovered)
            initialProvenance = 'previous_recovered_q';
        end
        seeds = build_local_seed_ladder(previousQ, initialProvenance, ...
            positionLookup, target.source_index, config, size(limits, 1));
        [accepted, point] = recover_one_target(model, gik, endEffector, ...
            limits, target, seeds, postureConfig, config);
        if accepted
            point.delta_q = point.q - previousQ;
            recovered(end + 1) = point; %#ok<AGROW>
            previousQ = point.q;
        else
            unrecovered(end + 1) = point; %#ok<AGROW>
        end
    end

    result.recovered_points = recovered;
    result.unrecovered_points = unrecovered;
    result.recovered_point_count = numel(recovered);
    result.remaining_failed_point_count = numel(unrecovered);
    result.fully_reconnected = isempty(unrecovered) && ...
        numel(recovered) == numel(window.original_writing_targets);
    if result.fully_reconnected
        result.reconnect_delta_q = window.right_accepted_q - previousQ;
        sequenceQ = [window.left_accepted_q; vertcat(recovered.q); ...
            window.right_accepted_q];
        result.max_joint_step_rad = max(abs(diff(sequenceQ, 1, 1)), [], 'all');
    else
        result.reconnect_delta_q = [];
        result.max_joint_step_rad = maximum_finite(abs(vertcat(recovered.delta_q)));
    end
    result.minimum_joint_limit_margin = minimum_margin( ...
        [window.left_accepted_q; recovered_q_or_empty(recovered, size(limits, 1)); ...
         window.right_accepted_q], limits);
end


function [accepted, point] = recover_one_target(model, gik, endEffector, limits, ...
        target, seeds, postureConfig, config)

    attempts = repmat(empty_attempt(), 1, 0);
    for toleranceIndex = 1:numel(config.direction_tolerance_ladder_deg)
        toleranceDeg = config.direction_tolerance_ladder_deg(toleranceIndex);
        toleranceRad = deg2rad(toleranceDeg);
        for seedIndex = 1:numel(seeds)
            attempt = solve_one_attempt(model, gik, endEffector, limits, target, ...
                seeds(seedIndex), postureConfig.position_tolerance_m, toleranceRad);
            attempt.direction_tolerance_deg = toleranceDeg;
            attempt.direction_tolerance_rad = toleranceRad;
            attempts(end + 1) = attempt; %#ok<AGROW>
            if attempt.accepted
                accepted = true;
                point = recovered_from_attempt(target, attempt, attempts, config);
                return;
            end
        end
    end
    accepted = false;
    point = unrecovered_from_attempts(target, attempts, config);
end


function attempt = solve_one_attempt(model, gik, endEffector, limits, target, seed, ...
        positionToleranceM, directionToleranceRad)

    attempt = empty_attempt();
    attempt.seed_q = seed.q;
    attempt.seed_source = seed.provenance;
    positionConstraint = constraintPositionTarget(endEffector);
    positionConstraint.TargetPosition = target.target_xyz_m;
    positionConstraint.PositionTolerance = positionToleranceM;
    aimingConstraint = constraintAiming(endEffector);
    aimingConstraint.TargetPoint = target.aim_target_m;
    aimingConstraint.AngularTolerance = directionToleranceRad;
    try
        [qCandidate, solutionInfo] = gik(seed.q, positionConstraint, aimingConstraint);
        qCandidate = reshape(qCandidate, 1, []);
        fkPose = getTransform(model, qCandidate, endEffector);
        attempt.candidate_q = qCandidate;
        attempt.fk_xyz_m = fkPose(1:3, 4)';
        attempt.fk_tool_z_direction = unit_vector(fkPose(1:3, 3)');
        attempt.position_error_m = norm(attempt.fk_xyz_m - target.target_xyz_m);
        attempt.tool_direction_error_rad = angle_between_unit_vectors( ...
            attempt.fk_tool_z_direction, target.target_tool_direction);
        attempt.tool_direction_error_deg = rad2deg(attempt.tool_direction_error_rad);
        [attempt.within_joint_limits, attempt.joint_limit_margin] = ...
            joint_limit_evidence(qCandidate, model, limits);
        attempt.solver_status = char(string(solutionInfo.Status));
        attempt.solver_status_is_success = strcmpi(attempt.solver_status, 'success');
        attempt.within_position_tolerance = ...
            attempt.position_error_m <= positionToleranceM;
        attempt.within_direction_tolerance = ...
            attempt.tool_direction_error_rad <= directionToleranceRad;
        attempt.accepted = attempt.solver_status_is_success && ...
            attempt.within_position_tolerance && attempt.within_direction_tolerance && ...
            attempt.within_joint_limits;
    catch exception
        attempt.solver_status = ['exception:' exception.identifier];
        attempt.exception_identifier = exception.identifier;
        attempt.exception_message = exception.message;
    end
end


function point = recovered_from_attempt(target, attempt, attempts, config)

    point = empty_recovered_point();
    point.source_index = target.source_index;
    point.t_ms = target.t_ms;
    point.original_target_xyz_m = target.target_xyz_m;
    point.original_target_tool_direction = target.target_tool_direction;
    point.aim_target_m = target.aim_target_m;
    point.original_strict_direction_tolerance_deg = ...
        config.strict_direction_tolerance_deg;
    point.final_relaxed_direction_tolerance_deg = attempt.direction_tolerance_deg;
    point.final_relaxed_direction_tolerance_rad = attempt.direction_tolerance_rad;
    point.q = attempt.candidate_q;
    point.solver_status = attempt.solver_status;
    point.fk_xyz_m = attempt.fk_xyz_m;
    point.position_error_m = attempt.position_error_m;
    point.fk_tool_z_direction = attempt.fk_tool_z_direction;
    point.tool_direction_error_rad = attempt.tool_direction_error_rad;
    point.tool_direction_error_deg = attempt.tool_direction_error_deg;
    point.within_joint_limits = attempt.within_joint_limits;
    point.joint_limit_margin = attempt.joint_limit_margin;
    point.seed_source = attempt.seed_source;
    point.attempt_count = numel(attempts);
    point.attempts = attempts;
    point.provenance = target.provenance;
    point.recovery_reason = config.recovery_reason;
    point.accepted = attempt.accepted;
    point.delta_q = [];
end


function point = unrecovered_from_attempts(target, attempts, config)

    point = empty_unrecovered_point();
    point.source_index = target.source_index;
    point.t_ms = target.t_ms;
    point.original_target_xyz_m = target.target_xyz_m;
    point.original_target_tool_direction = target.target_tool_direction;
    point.aim_target_m = target.aim_target_m;
    point.original_strict_direction_tolerance_deg = ...
        config.strict_direction_tolerance_deg;
    point.attempt_count = numel(attempts);
    point.attempts = attempts;
    point.provenance = target.provenance;
    point.recovery_reason = config.recovery_reason;
    if isempty(attempts), return; end
    last = attempts(end);
    point.final_attempt_direction_tolerance_deg = last.direction_tolerance_deg;
    point.final_attempt_position_error_m = last.position_error_m;
    point.final_attempt_direction_error_deg = last.tool_direction_error_deg;
    point.final_attempt_solver_status = last.solver_status;
    point.final_attempt_seed_source = last.seed_source;
end


function seeds = build_local_seed_ladder(previousQ, previousProvenance, lookup, ...
        sourceIndex, config, dof)

    seeds = repmat(struct('provenance', '', 'q', NaN(1, dof)), 1, 0);
    seeds = append_seed(seeds, previousProvenance, previousQ, config, dof);
    match = find(lookup.source_indices == sourceIndex, 1, 'first');
    if ~isempty(match)
        seeds = append_seed(seeds, 'same_source_position_only_q', ...
            lookup.q_rad(match, :), config, dof);
    end
end


function seeds = append_seed(seeds, provenance, q, config, dof)

    q = reshape(q, 1, []);
    if ~isequal(size(q), [1, dof]) || any(~isfinite(q))
        error('MonoTeach:InvalidConstrainedGapRecoverySeed', ...
            'Recovery seed must be a finite row matching RobotContext.dof.');
    end
    if any(arrayfun(@(seed) norm(seed.q - q) <= ...
            config.duplicate_seed_tolerance_rad, seeds))
        return;
    end
    seeds(end + 1) = struct('provenance', provenance, 'q', q); %#ok<AGROW>
end


function gik = configured_gik(model, algorithm, allowRandomRestart)

    gik = generalizedInverseKinematics('RigidBodyTree', model, ...
        'ConstraintInputs', {'position', 'aiming'}, 'SolverAlgorithm', algorithm);
    parameters = gik.SolverParameters;
    parameters.AllowRandomRestart = allowRandomRestart;
    gik.SolverParameters = parameters;
end


function [inside, margin] = joint_limit_evidence(q, model, limits)

    dof = numel(homeConfiguration(model));
    if ~isequal(size(q), [1, dof]) || ~isequal(size(limits), [dof, 2])
        error('MonoTeach:InvalidConstrainedGapRecoveryConfiguration', ...
            'Joint configuration and RobotContext limits must match model DOF.');
    end
    margin = min(q - limits(:, 1)', limits(:, 2)' - q);
    inside = all(q >= limits(:, 1)' & q <= limits(:, 2)');
end


function value = unit_vector(vector)

    magnitude = norm(vector);
    if ~isfinite(magnitude) || magnitude <= eps
        error('MonoTeach:DegenerateConstrainedGapToolDirection', ...
            'FK tool direction must be finite and nonzero.');
    end
    value = vector / magnitude;
end


function value = angle_between_unit_vectors(first, second)

    value = acos(max(-1.0, min(1.0, dot(first, second))));
end


function q = recovered_q_or_empty(points, dof)

    if isempty(points), q = zeros(0, dof); else, q = vertcat(points.q); end
end


function value = maximum_finite(values)

    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = max(values); end
end


function value = minimum_margin(q, limits)

    if isempty(q), value = NaN; else, value = min( ...
        min(q - limits(:, 1)', limits(:, 2)' - q), [], 'all'); end
end


function summary = summarize_recovery(results)

    recoveredCount = sum([results.recovered_point_count]);
    remainingCount = sum([results.remaining_failed_point_count]);
    summary = struct('window_count', numel(results), ...
        'recovered_point_count', recoveredCount, ...
        'remaining_failed_point_count', remainingCount, ...
        'fully_reconnected_gap_count', sum([results.fully_reconnected]), ...
        'incomplete_gap_count', sum(~[results.fully_reconnected]));
end


function value = empty_attempt()

    value = struct('seed_q', [], 'seed_source', '', 'candidate_q', [], ...
        'solver_status', '', 'fk_xyz_m', NaN(1, 3), ...
        'fk_tool_z_direction', NaN(1, 3), 'position_error_m', NaN, ...
        'tool_direction_error_rad', NaN, 'tool_direction_error_deg', NaN, ...
        'joint_limit_margin', [], 'within_joint_limits', false, ...
        'within_position_tolerance', false, 'within_direction_tolerance', false, ...
        'solver_status_is_success', false, 'accepted', false, ...
        'direction_tolerance_deg', NaN, 'direction_tolerance_rad', NaN, ...
        'exception_identifier', '', 'exception_message', '');
end


function value = empty_recovered_point()

    value = struct('source_index', NaN, 't_ms', NaN, ...
        'original_target_xyz_m', NaN(1, 3), ...
        'original_target_tool_direction', NaN(1, 3), 'aim_target_m', NaN(1, 3), ...
        'original_strict_direction_tolerance_deg', NaN, ...
        'final_relaxed_direction_tolerance_deg', NaN, ...
        'final_relaxed_direction_tolerance_rad', NaN, 'q', [], ...
        'solver_status', '', 'fk_xyz_m', NaN(1, 3), ...
        'position_error_m', NaN, 'fk_tool_z_direction', NaN(1, 3), ...
        'tool_direction_error_rad', NaN, 'tool_direction_error_deg', NaN, ...
        'within_joint_limits', false, 'joint_limit_margin', [], ...
        'seed_source', '', 'attempt_count', 0, 'attempts', struct([]), ...
        'provenance', struct([]), 'recovery_reason', '', 'accepted', false, ...
        'delta_q', []);
end


function value = empty_unrecovered_point()

    value = struct('source_index', NaN, 't_ms', NaN, ...
        'original_target_xyz_m', NaN(1, 3), ...
        'original_target_tool_direction', NaN(1, 3), 'aim_target_m', NaN(1, 3), ...
        'original_strict_direction_tolerance_deg', NaN, 'attempt_count', 0, ...
        'attempts', struct([]), 'provenance', struct([]), 'recovery_reason', '', ...
        'final_attempt_direction_tolerance_deg', NaN, ...
        'final_attempt_position_error_m', NaN, ...
        'final_attempt_direction_error_deg', NaN, ...
        'final_attempt_solver_status', '', 'final_attempt_seed_source', '');
end


function value = empty_window_result()

    value = struct('gap_index', NaN, 'preik_segment_index', NaN, ...
        'original_run_index', NaN, 'left_success_index', NaN, ...
        'right_success_index', NaN, 'failed_indices', zeros(1, 0), ...
        'strict_failure_reasons', {{}}, 'left_accepted_q', [], ...
        'right_accepted_q', [], 'original_writing_targets', struct([]), ...
        'recovered_points', struct([]), 'unrecovered_points', struct([]), ...
        'recovered_point_count', 0, 'remaining_failed_point_count', 0, ...
        'fully_reconnected', false, 'reconnect_delta_q', [], ...
        'max_joint_step_rad', NaN, 'minimum_joint_limit_margin', NaN);
end


function validate_inputs(context, windows, posture, config, lookup)

    requiredContext = {'id', 'model', 'dof', 'end_effector', 'joint_limits', 'tool'};
    requiredConfig = {'strict_direction_tolerance_deg', ...
        'max_recovery_direction_tolerance_deg', 'direction_tolerance_ladder_deg', ...
        'position_tolerance_m', 'solver_algorithm', 'allow_random_restart', ...
        'duplicate_seed_tolerance_rad'};
    if ~isstruct(context) || ~all(isfield(context, requiredContext)) || ...
            ~isa(context.model, 'rigidBodyTree') || ...
            ~strcmp(context.model.DataFormat, 'row') || ...
            context.dof ~= numel(homeConfiguration(context.model)) || ...
            ~isequal(size(context.joint_limits), [context.dof, 2]) || ...
            ~strcmp(string(context.tool.axis), "z") || ...
            ~isstruct(windows) || ~isstruct(posture) || ...
            ~isfield(posture, 'end_effector') || ~isfield(posture, 'tool_axis') || ...
            ~strcmp(string(posture.end_effector), string(context.end_effector)) || ...
            ~strcmp(string(posture.tool_axis), string(context.tool.axis)) || ...
            ~isstruct(config) || ~all(isfield(config, requiredConfig)) || ...
            config.allow_random_restart || ...
            config.position_tolerance_m ~= posture.position_tolerance_m || ...
            ~isstruct(lookup) || ~isfield(lookup, 'source_indices') || ...
            ~isfield(lookup, 'q_rad') || size(lookup.q_rad, 2) ~= context.dof
        error('MonoTeach:InvalidConstrainedGapRecoveryInput', ...
            'Recovery requires a deterministic RobotContext and frozen Writing contract.');
    end
    if any(diff(config.direction_tolerance_ladder_deg) <= 0) || ...
            config.direction_tolerance_ladder_deg(1) ~= ...
                config.strict_direction_tolerance_deg || ...
            config.direction_tolerance_ladder_deg(end) > ...
                config.max_recovery_direction_tolerance_deg
        error('MonoTeach:InvalidConstrainedGapRecoveryLadder', ...
            'Recovery direction tolerance ladder must be monotonic and bounded.');
    end
end


function validate_window(window, dof)

    if ~isstruct(window) || ~isfield(window, 'original_writing_targets') || ...
            ~isfield(window, 'left_accepted_q') || ...
            ~isfield(window, 'right_accepted_q') || ...
            ~isequal(size(window.left_accepted_q), [1, dof]) || ...
            ~isequal(size(window.right_accepted_q), [1, dof]) || ...
            numel(window.failed_indices) ~= numel(window.original_writing_targets)
        error('MonoTeach:InvalidConstrainedGapRecoveryWindow', ...
            'RecoveryWindow must contain bracketed successes and failed targets.');
    end
end
