function resultSet = solve_writing_ik_segments(robot, segmentSet, centerSeed, ...
        taskPlaneConfig, postureConfig, candidateConfig, positionLookup, policy)
%SOLVE_WRITING_IK_SEGMENTS Deterministic continuous XYZ + local-Z Writing IK.
% Failure is a hard barrier.  Every target tries an ordered, deduplicated
% deterministic seed ladder; the first accepted attempt stops that ladder.

    if nargin < 7 || isempty(positionLookup)
        positionLookup = empty_lookup();
    end
    if nargin < 8 || isempty(policy)
        policy = default_writing_recovery_policy('position_only_recovery');
    end
    validate_inputs(robot, segmentSet, centerSeed, taskPlaneConfig, ...
        postureConfig, candidateConfig, positionLookup, policy);

    endEffector = char(string(postureConfig.end_effector));
    signedNormal = candidateConfig.normal_sign * ...
        derive_task_plane_normal(taskPlaneConfig)';
    limits = five_joint_limits(robot);
    homeQ = row_configuration(homeConfiguration(robot));
    gik = configured_gik(robot, postureConfig.solver_algorithm, ...
        postureConfig.allow_random_restart);
    successSegments = repmat(empty_success_segment(), 1, 0);
    failures = repmat(empty_failure(), 1, 0);

    for segmentIndex = 1:numel(segmentSet.segments)
        preIkSegment = segmentSet.segments(segmentIndex);
        previousQ = [];
        currentResults = struct([]);
        for sampleIndex = 1:numel(preIkSegment.samples)
            sample = preIkSegment.samples(sampleIndex);
            targetXYZM = [sample.x_m, sample.y_m, sample.z_m];
            aimTargetM = targetXYZM + postureConfig.aim_distance_m * signedNormal;
            sourceIndex = preIkSegment.source_indices(sampleIndex);
            seeds = build_writing_recovery_ladder(previousQ, positionLookup, ...
                sourceIndex, centerSeed, homeQ, policy);
            [accepted, point, failure] = solve_target_ladder(robot, gik, ...
                endEffector, targetXYZM, aimTargetM, signedNormal, seeds, limits, ...
                postureConfig, segmentIndex, sourceIndex, sample.t_ms);
            if accepted
                if isempty(previousQ)
                    point.delta_q = [];
                else
                    point.delta_q = point.q - previousQ;
                end
                currentResults = append_result(currentResults, point);
                previousQ = point.q;
            else
                failures(end + 1) = failure; %#ok<AGROW>
                if ~isempty(currentResults)
                    successSegments(end + 1) = build_success_segment( ...
                        segmentIndex, currentResults); %#ok<AGROW>
                end
                currentResults = struct([]);
                previousQ = [];
            end
        end
        if ~isempty(currentResults)
            successSegments(end + 1) = build_success_segment( ...
                segmentIndex, currentResults); %#ok<AGROW>
        end
    end

    resultSet = struct('coordinate_space', 'joint', 'units', 'rad', ...
        'task_definition', 'xyz_plus_body5_local_z_aiming', ...
        'end_effector', endEffector, 'tool_axis', 'body5_local_z', ...
        'signed_normal_base', signedNormal, 'restart_policy', policy.name, ...
        'recovery_policy', policy, 'position_only_seed_lookup', positionLookup, ...
        'center_seed', centerSeed, 'segments', successSegments, ...
        'failures', failures);
    resultSet.summary = summarize_result_set(successSegments, failures);
end


function [accepted, point, failure] = solve_target_ladder(robot, gik, ...
        endEffector, targetXYZM, aimTargetM, signedNormal, seeds, limits, ...
        config, segmentIndex, sourceIndex, tMs)

    attempts = repmat(empty_attempt(), 1, 0);
    for index = 1:numel(seeds)
        attempt = solve_one_attempt(robot, gik, endEffector, targetXYZM, ...
            aimTargetM, signedNormal, seeds(index), limits, config);
        attempts(end + 1) = attempt; %#ok<AGROW>
        if attempt.accepted
            accepted = true;
            point = point_from_attempt(attempt, attempts, segmentIndex, ...
                sourceIndex, tMs, targetXYZM, aimTargetM, signedNormal);
            failure = empty_failure();
            return;
        end
    end
    accepted = false;
    point = struct([]);
    failure = failure_from_attempts(attempts, segmentIndex, sourceIndex, ...
        tMs, targetXYZM, aimTargetM, signedNormal);
end


function attempt = solve_one_attempt(robot, gik, endEffector, targetXYZM, ...
        aimTargetM, signedNormal, seed, limits, config)

    attempt = empty_attempt();
    attempt.seed_q = seed.q;
    attempt.seed_provenance = seed.provenance;
    positionConstraint = constraintPositionTarget(endEffector);
    positionConstraint.TargetPosition = targetXYZM;
    positionConstraint.PositionTolerance = config.position_tolerance_m;
    aimingConstraint = constraintAiming(endEffector);
    aimingConstraint.TargetPoint = aimTargetM;
    aimingConstraint.AngularTolerance = config.aiming_angular_tolerance_rad;
    try
        [qCandidate, solutionInfo] = gik(seed.q, positionConstraint, aimingConstraint);
        qCandidate = row_configuration(qCandidate);
        fkPose = getTransform(robot, qCandidate, endEffector);
        attempt.candidate_q = qCandidate;
        attempt.fk_xyz_m = fkPose(1:3, 4)';
        attempt.fk_tool_z_direction = unit_vector(fkPose(1:3, 3)');
        [attempt.within_joint_limits, attempt.joint_limit_margin] = ...
            joint_limit_evidence(qCandidate, limits);
        attempt.position_error_m = norm(attempt.fk_xyz_m - targetXYZM);
        attempt.tool_direction_error_rad = angle_between_unit_vectors( ...
            attempt.fk_tool_z_direction, signedNormal);
        attempt.tool_direction_error_deg = rad2deg(attempt.tool_direction_error_rad);
        attempt.solver_status = char(string(solutionInfo.Status));
        attempt.within_position_tolerance = ...
            attempt.position_error_m <= config.position_tolerance_m;
        attempt.within_aiming_tolerance = ...
            attempt.tool_direction_error_rad <= config.aiming_angular_tolerance_rad;
        attempt.solver_status_is_success = strcmpi(attempt.solver_status, 'success');
        attempt.accepted = attempt.solver_status_is_success && ...
            attempt.within_position_tolerance && ...
            attempt.within_aiming_tolerance && attempt.within_joint_limits;
    catch exception
        attempt.solver_status = ['exception:' exception.identifier];
        attempt.exception_identifier = exception.identifier;
        attempt.exception_message = exception.message;
    end
end


function point = point_from_attempt(attempt, attempts, segmentIndex, ...
        sourceIndex, tMs, targetXYZM, aimTargetM, signedNormal)

    point = empty_point();
    point.preik_segment_index = segmentIndex;
    point.source_index = sourceIndex;
    point.t_ms = tMs;
    point.target_xyz_m = targetXYZM;
    point.aim_target_m = aimTargetM;
    point.signed_normal_base = signedNormal;
    point.seed_q = attempt.seed_q;
    point.seed_provenance = attempt.seed_provenance;
    point.accepted_seed_provenance = attempt.seed_provenance;
    point.attempt_count = numel(attempts);
    point.attempted_seed_provenances = {attempts.seed_provenance};
    point.attempts = attempts;
    point.q = attempt.candidate_q;
    point.solver_status = attempt.solver_status;
    point.fk_xyz_m = attempt.fk_xyz_m;
    point.position_error_m = attempt.position_error_m;
    point.fk_tool_z_direction = attempt.fk_tool_z_direction;
    point.tool_direction_error_rad = attempt.tool_direction_error_rad;
    point.tool_direction_error_deg = attempt.tool_direction_error_deg;
    point.within_joint_limits = attempt.within_joint_limits;
    point.joint_limit_margin = attempt.joint_limit_margin;
    point.within_position_tolerance = attempt.within_position_tolerance;
    point.within_aiming_tolerance = attempt.within_aiming_tolerance;
    point.solver_status_is_success = attempt.solver_status_is_success;
    point.accepted = attempt.accepted;
end


function failure = failure_from_attempts(attempts, segmentIndex, sourceIndex, ...
        tMs, targetXYZM, aimTargetM, signedNormal)

    failure = empty_failure();
    failure.preik_segment_index = segmentIndex;
    failure.source_index = sourceIndex;
    failure.t_ms = tMs;
    failure.target_xyz_m = targetXYZM;
    failure.aim_target_m = aimTargetM;
    failure.signed_normal_base = signedNormal;
    failure.attempt_count = numel(attempts);
    failure.attempted_seed_provenances = {attempts.seed_provenance};
    failure.attempts = attempts;
    failure.attempt_position_errors_m = [attempts.position_error_m];
    failure.attempt_direction_errors_rad = [attempts.tool_direction_error_rad];
    failure.attempt_direction_errors_deg = [attempts.tool_direction_error_deg];
    failure.attempt_solver_statuses = {attempts.solver_status};
    if isempty(attempts)
        failure.solver_status = 'no_recovery_seed_available';
        failure.reason = 'no_recovery_seed_available';
        return;
    end
    % This is evidence from the final attempted seed, never a success q.
    last = attempts(end);
    failure.seed_q = last.seed_q;
    failure.seed_provenance = last.seed_provenance;
    failure.solver_status = last.solver_status;
    failure.candidate_q = last.candidate_q;
    failure.fk_xyz_m = last.fk_xyz_m;
    failure.position_error_m = last.position_error_m;
    failure.tool_direction_error_rad = last.tool_direction_error_rad;
    failure.tool_direction_error_deg = last.tool_direction_error_deg;
    failure.within_joint_limits = last.within_joint_limits;
    failure.joint_limit_margin = last.joint_limit_margin;
    failure.exception_identifier = last.exception_identifier;
    failure.exception_message = last.exception_message;
    failure.reason = classify_failed_attempt(last);
end


function reason = classify_failed_attempt(attempt)
    if startsWith(attempt.solver_status, 'exception:')
        reason = 'solver_or_fk_exception';
    elseif ~attempt.solver_status_is_success
        reason = 'solver_status_not_success';
    elseif ~attempt.within_joint_limits
        reason = 'joint_limit_violation';
    elseif ~attempt.within_position_tolerance
        reason = 'position_error_exceeds_tolerance';
    elseif ~attempt.within_aiming_tolerance
        reason = 'tool_direction_error_exceeds_tolerance';
    else
        reason = 'unknown_acceptance_failure';
    end
end


function gik = configured_gik(robot, algorithm, allowRandomRestart)
    gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', {'position', 'aiming'}, 'SolverAlgorithm', algorithm);
    parameters = gik.SolverParameters;
    if ~isfield(parameters, 'AllowRandomRestart')
        error('MonoTeach:UnsupportedWritingGIKAPI', ...
            'R2024a GIK SolverParameters must contain AllowRandomRestart.');
    end
    parameters.AllowRandomRestart = allowRandomRestart;
    gik.SolverParameters = parameters;
end


function summary = summarize_result_set(segments, failures)
    results = struct([]);
    for index = 1:numel(segments)
        if isempty(results)
            results = segments(index).results;
        else
            results = [results, segments(index).results]; %#ok<AGROW>
        end
    end
    preIkIndices = [segments.preik_segment_index, failures.preik_segment_index];
    summary = struct('preik_segment_count', numel(unique(preIkIndices)), ...
        'ik_success_segment_count', numel(segments), ...
        'success_point_count', numel(results), 'failure_count', numel(failures));
    summary.ik_success_rate = safe_ratio(summary.success_point_count, ...
        summary.success_point_count + summary.failure_count);
    summary.failure_source_indices = [failures.source_index];
    summary.failure_reasons = {failures.reason};
    summary.failure_reason_distribution = label_distribution(summary.failure_reasons);
    summary.position_tolerance_failure_error_distribution = ...
        position_failure_distribution(failures);
    if isempty(results)
        summary.mean_fk_position_error_m = NaN; summary.max_fk_position_error_m = NaN;
        summary.mean_tool_direction_error_rad = NaN; summary.max_tool_direction_error_rad = NaN;
        summary.mean_tool_direction_error_deg = NaN; summary.max_tool_direction_error_deg = NaN;
        summary.overall_max_joint_step = NaN; summary.minimum_joint_limit_margin = NaN;
        summary.accepted_seed_provenance_distribution = label_distribution({});
        return;
    end
    summary.mean_fk_position_error_m = mean([results.position_error_m]);
    summary.max_fk_position_error_m = max([results.position_error_m]);
    summary.mean_tool_direction_error_rad = mean([results.tool_direction_error_rad]);
    summary.max_tool_direction_error_rad = max([results.tool_direction_error_rad]);
    summary.mean_tool_direction_error_deg = mean([results.tool_direction_error_deg]);
    summary.max_tool_direction_error_deg = max([results.tool_direction_error_deg]);
    summary.minimum_joint_limit_margin = min(vertcat(results.joint_limit_margin), [], 'all');
    deltaQ = vertcat(results.delta_q);
    if isempty(deltaQ), summary.overall_max_joint_step = NaN;
    else, summary.overall_max_joint_step = max(abs(deltaQ), [], 'all'); end
    summary.accepted_seed_provenance_distribution = ...
        label_distribution({results.accepted_seed_provenance});
end


function distribution = position_failure_distribution(failures)
    errors = [failures.position_error_m];
    mask = strcmp({failures.reason}, 'position_error_exceeds_tolerance') & isfinite(errors);
    errors = errors(mask);
    distribution = struct('count', numel(errors), 'errors_m', errors, ...
        'min_m', NaN, 'median_m', NaN, 'max_m', NaN, ...
        'count_1e4_to_1p5e4', 0, 'count_greater_than_1p5e4', 0);
    if isempty(errors), return; end
    distribution.min_m = min(errors); distribution.median_m = median(errors);
    distribution.max_m = max(errors);
    distribution.count_1e4_to_1p5e4 = sum(errors >= 1e-4 & errors <= 1.5e-4);
    distribution.count_greater_than_1p5e4 = sum(errors > 1.5e-4);
end


function distribution = label_distribution(labels)
    distribution = struct('labels', {{}}, 'counts', zeros(1, 0));
    if isempty(labels), return; end
    [names, ~, groups] = unique(labels, 'stable');
    distribution.labels = names;
    distribution.counts = accumarray(groups(:), 1)';
end


function value = safe_ratio(numerator, denominator)
    if denominator == 0, value = NaN; else, value = numerator / denominator; end
end


function array = append_result(array, result)
    if isempty(array), array = result; else, array(end + 1) = result; end %#ok<AGROW>
end


function segment = build_success_segment(index, results)
    segment = struct('preik_segment_index', index, ...
        'source_indices', [results.source_index], 't_ms', [results.t_ms], ...
        'q_rad', vertcat(results.q), 'results', results);
end


function segment = empty_success_segment()
    segment = struct('preik_segment_index', [], 'source_indices', [], ...
        't_ms', [], 'q_rad', [], 'results', struct([]));
end


function attempt = empty_attempt()
    attempt = struct('seed_q', NaN(1,5), 'seed_provenance', '', ...
        'candidate_q', NaN(1,5), 'solver_status', '', 'fk_xyz_m', NaN(1,3), ...
        'position_error_m', NaN, 'fk_tool_z_direction', NaN(1,3), ...
        'tool_direction_error_rad', NaN, 'tool_direction_error_deg', NaN, ...
        'within_joint_limits', false, 'joint_limit_margin', NaN(1,5), ...
        'within_position_tolerance', false, 'within_aiming_tolerance', false, ...
        'solver_status_is_success', false, 'accepted', false, ...
        'exception_identifier', '', 'exception_message', '');
end


function point = empty_point()
    point = struct('preik_segment_index', NaN, 'source_index', NaN, 't_ms', NaN, ...
        'target_xyz_m', NaN(1,3), 'aim_target_m', NaN(1,3), ...
        'signed_normal_base', NaN(1,3), 'seed_q', NaN(1,5), ...
        'seed_provenance', '', 'accepted_seed_provenance', '', ...
        'attempt_count', 0, 'attempted_seed_provenances', {{}}, ...
        'attempts', struct([]), 'q', NaN(1,5), 'solver_status', '', ...
        'fk_xyz_m', NaN(1,3), 'position_error_m', NaN, ...
        'fk_tool_z_direction', NaN(1,3), 'tool_direction_error_rad', NaN, ...
        'tool_direction_error_deg', NaN, 'within_joint_limits', false, ...
        'joint_limit_margin', NaN(1,5), 'within_position_tolerance', false, ...
        'within_aiming_tolerance', false, 'solver_status_is_success', false, ...
        'accepted', false, 'delta_q', []);
end


function failure = empty_failure()
    failure = struct('preik_segment_index', NaN, 'source_index', NaN, 't_ms', NaN, ...
        'target_xyz_m', NaN(1,3), 'aim_target_m', NaN(1,3), ...
        'signed_normal_base', NaN(1,3), 'seed_q', NaN(1,5), ...
        'seed_provenance', '', 'attempt_count', 0, ...
        'attempted_seed_provenances', {{}}, 'attempts', struct([]), ...
        'attempt_position_errors_m', zeros(1,0), ...
        'attempt_direction_errors_rad', zeros(1,0), ...
        'attempt_direction_errors_deg', zeros(1,0), ...
        'attempt_solver_statuses', {{}}, 'solver_status', '', 'q', [], ...
        'candidate_q', [], 'fk_xyz_m', NaN(1,3), 'position_error_m', NaN, ...
        'tool_direction_error_rad', NaN, 'tool_direction_error_deg', NaN, ...
        'within_joint_limits', false, 'joint_limit_margin', NaN(1,5), ...
        'reason', '', 'exception_identifier', '', 'exception_message', '');
end


function lookup = empty_lookup()
    lookup = struct('source_indices', zeros(1,0), 'q_rad', zeros(0,5), ...
        'provenance', 'same_source_position_only_q');
end


function validate_inputs(robot, segmentSet, centerSeed, taskPlaneConfig, config, ...
        candidateConfig, lookup, policy)
    if ~isa(robot, 'rigidBodyTree') || ~strcmp(robot.DataFormat, 'row') || ...
            robot.NumBodies ~= 5 || ~isstruct(segmentSet) || ...
            ~isfield(segmentSet, 'segments') || ~isstruct(centerSeed) || ...
            ~isfield(centerSeed, 'q') || ~isequal(size(centerSeed.q), [1,5]) || ...
            any(~isfinite(centerSeed.q)) || ~isstruct(taskPlaneConfig) || ...
            ~isstruct(config) || ~strcmp(string(config.end_effector), "body5") || ...
            ~strcmp(string(config.tool_axis), "z") || candidateConfig.normal_sign ~= 1 || ...
            ~strcmp(string(config.solver_algorithm), "BFGSGradientProjection") || ...
            config.allow_random_restart || ~isstruct(lookup) || ...
            ~isfield(lookup, 'source_indices') || ~isfield(lookup, 'q_rad') || ...
            ~isstruct(policy) || ~isfield(policy, 'name')
        error('MonoTeach:InvalidContinuousWritingIKInput', ...
            'Writing IK requires approved deterministic candidate inputs.');
    end
end


function q = row_configuration(configuration)
    q = reshape(configuration, 1, []);
    if ~isnumeric(q) || ~isreal(q) || ~isequal(size(q), [1,5]) || any(~isfinite(q))
        error('MonoTeach:InvalidContinuousWritingConfiguration', ...
            'Writing IK configurations must be finite 1-by-5 rows.');
    end
end


function limits = five_joint_limits(robot)
    limits = zeros(5,2);
    for index = 1:5, limits(index,:) = robot.Bodies{index}.Joint.PositionLimits; end
end


function [inside, margin] = joint_limit_evidence(q, limits)
    margin = min(q - limits(:,1)', limits(:,2)' - q);
    inside = all(q >= limits(:,1)' & q <= limits(:,2)');
end


function value = unit_vector(vector)
    magnitude = norm(vector);
    if ~isfinite(magnitude) || magnitude <= eps
        error('MonoTeach:DegenerateContinuousWritingToolDirection', ...
            'FK body5 local-Z direction must be finite and nonzero.');
    end
    value = vector / magnitude;
end


function angleRad = angle_between_unit_vectors(first, second)
    angleRad = acos(max(-1.0, min(1.0, dot(first, second))));
end
