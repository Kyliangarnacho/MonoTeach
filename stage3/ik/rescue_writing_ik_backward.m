function rescued = rescue_writing_ik_backward(robot, segmentSet, forwardResultSet, ...
        taskPlaneConfig, postureConfig, candidateConfig)
%RESCUE_WRITING_IK_BACKWARD Offline, failure-only backward Writing IK pass.
%
% The forward result is retained unchanged.  Only a forward failure with a
% later success in the same Pre-IK segment receives one additional seed:
% that next accepted Writing q.  This is an offline numerical rescue, never
% an instruction to execute a motion across a failure barrier.

    validate_inputs(robot, segmentSet, forwardResultSet, taskPlaneConfig, ...
        postureConfig, candidateConfig);
    plan = build_backward_writing_rescue_plan(segmentSet, forwardResultSet);
    endEffector = char(string(postureConfig.end_effector));
    signedNormal = candidateConfig.normal_sign * ...
        derive_task_plane_normal(taskPlaneConfig)';
    limits = five_joint_limits(robot);
    gik = configured_gik(robot, postureConfig.solver_algorithm, ...
        postureConfig.allow_random_restart);
    forwardPoints = flatten_successes(forwardResultSet.segments);
    forwardFailures = forwardResultSet.failures;
    templatePoint = forwardPoints(1);
    rescuedSegments = repmat(empty_segment(), 1, 0);
    remainingFailures = struct([]);
    rescueAttempts = repmat(empty_rescue_record(), 1, 0);

    for segmentIndex = 1:numel(segmentSet.segments)
        segment = segmentSet.segments(segmentIndex);
        points = cell(1, numel(segment.samples));
        failures = cell(1, numel(segment.samples));
        for index = 1:numel(segment.samples)
            sourceIndex = segment.source_indices(index);
            pointIndex = find([forwardPoints.source_index] == sourceIndex, 1, 'first');
            if ~isempty(pointIndex)
                points{index} = normalize_forward_point(forwardPoints(pointIndex));
            else
                failureIndex = find([forwardFailures.source_index] == sourceIndex, 1, 'first');
                if isempty(failureIndex)
                    error('MonoTeach:IncompleteForwardWritingResultSet', ...
                        'Every Pre-IK sample requires forward evidence.');
                end
                failures{index} = enrich_failure(forwardFailures(failureIndex), ...
                    false, NaN, [], struct([]));
            end
        end

        nextSuccessQ = [];
        nextSuccessSourceIndex = NaN;
        for index = numel(segment.samples):-1:1
            if ~isempty(points{index})
                nextSuccessQ = points{index}.q;
                nextSuccessSourceIndex = points{index}.source_index;
                continue;
            end
            failure = failures{index};
            if isempty(nextSuccessQ)
                continue;
            end
            sample = segment.samples(index);
            targetXYZM = [sample.x_m, sample.y_m, sample.z_m];
            aimTargetM = targetXYZM + postureConfig.aim_distance_m * signedNormal;
            attempt = solve_backward_attempt(robot, gik, endEffector, targetXYZM, ...
                aimTargetM, signedNormal, nextSuccessQ, limits, postureConfig);
            rescueRecord = empty_rescue_record();
            rescueRecord.preik_segment_index = segmentIndex;
            rescueRecord.source_index = segment.source_indices(index);
            rescueRecord.next_success_source_index = nextSuccessSourceIndex;
            rescueRecord.next_success_q = nextSuccessQ;
            rescueRecord.provenance = 'next_success_writing_q';
            rescueRecord.attempt = attempt;
            rescueRecord.accepted = attempt.accepted;
            rescueAttempts(end + 1) = rescueRecord; %#ok<AGROW>
            if attempt.accepted
                points{index} = point_from_backward_attempt(templatePoint, attempt, ...
                    segmentIndex, segment.source_indices(index), sample.t_ms, ...
                    targetXYZM, aimTargetM, signedNormal);
                nextSuccessQ = points{index}.q;
                nextSuccessSourceIndex = points{index}.source_index;
                failures{index} = [];
            else
                failures{index} = enrich_failure(failure, true, ...
                    nextSuccessSourceIndex, nextSuccessQ, attempt);
            end
        end

        current = struct([]);
        for index = 1:numel(points)
            if ~isempty(points{index})
                point = points{index};
                if isempty(current)
                    point.delta_q = [];
                else
                    point.delta_q = point.q - current(end).q;
                end
                current = append_point(current, point);
                continue;
            end
            if ~isempty(current)
                rescuedSegments(end + 1) = make_segment(segmentIndex, current); %#ok<AGROW>
            end
            current = struct([]);
            if isempty(remainingFailures)
                remainingFailures = failures{index};
            else
                remainingFailures(end + 1) = failures{index}; %#ok<AGROW>
            end
        end
        if ~isempty(current)
            rescuedSegments(end + 1) = make_segment(segmentIndex, current); %#ok<AGROW>
        end
    end

    resultSet = forwardResultSet;
    resultSet.segments = rescuedSegments;
    resultSet.failures = remainingFailures;
    resultSet.restart_policy = [forwardResultSet.restart_policy ...
        '+backward_failure_rescue'];
    resultSet.backward_rescue_plan = plan;
    resultSet.backward_rescue_attempts = rescueAttempts;
    resultSet.summary = summarize_result_set(rescuedSegments, remainingFailures);
    rescued = struct('forward_result_set', forwardResultSet, ...
        'forward_summary', forwardResultSet.summary, 'result_set', resultSet, ...
        'summary', resultSet.summary, 'backward_rescue_plan', plan, ...
        'backward_rescue_attempts', rescueAttempts, ...
        'backward_rescued_source_indices', ...
            [rescueAttempts([rescueAttempts.accepted]).source_index]);
end


function attempt = solve_backward_attempt(robot, gik, endEffector, targetXYZM, ...
        aimTargetM, signedNormal, seedQ, limits, config)

    attempt = empty_attempt();
    attempt.seed_q = seedQ;
    attempt.seed_provenance = 'next_success_writing_q';
    positionConstraint = constraintPositionTarget(endEffector);
    positionConstraint.TargetPosition = targetXYZM;
    positionConstraint.PositionTolerance = config.position_tolerance_m;
    aimingConstraint = constraintAiming(endEffector);
    aimingConstraint.TargetPoint = aimTargetM;
    aimingConstraint.AngularTolerance = config.aiming_angular_tolerance_rad;
    try
        [qCandidate, solutionInfo] = gik(seedQ, positionConstraint, aimingConstraint);
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


function point = normalize_forward_point(point)
    point.forward_accepted = true;
    point.backward_rescued = false;
    point.recovery_provenance = point.accepted_seed_provenance;
end


function point = point_from_backward_attempt(template, attempt, segmentIndex, ...
        sourceIndex, tMs, targetXYZM, aimTargetM, signedNormal)
    point = template;
    point.preik_segment_index = segmentIndex;
    point.source_index = sourceIndex;
    point.t_ms = tMs;
    point.target_xyz_m = targetXYZM;
    point.aim_target_m = aimTargetM;
    point.signed_normal_base = signedNormal;
    point.seed_q = attempt.seed_q;
    point.seed_provenance = 'next_success_writing_q';
    point.accepted_seed_provenance = 'next_success_writing_q';
    point.attempt_count = 1;
    point.attempted_seed_provenances = {'next_success_writing_q'};
    point.attempts = attempt;
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
    point.delta_q = [];
    point.forward_accepted = false;
    point.backward_rescued = true;
    point.recovery_provenance = 'backward_continuation';
end


function failure = enrich_failure(failure, attempted, nextSourceIndex, nextQ, backwardAttempt)
    failure.backward_rescue_attempted = attempted;
    failure.backward_rescue_provenance = 'next_success_writing_q';
    failure.next_success_source_index = nextSourceIndex;
    failure.next_success_q = nextQ;
    failure.backward_attempt = backwardAttempt;
    allAttempts = failure.attempts;
    if attempted
        allAttempts = [allAttempts, backwardAttempt]; %#ok<AGROW>
        failure.solver_status = backwardAttempt.solver_status;
        failure.candidate_q = backwardAttempt.candidate_q;
        failure.fk_xyz_m = backwardAttempt.fk_xyz_m;
        failure.position_error_m = backwardAttempt.position_error_m;
        failure.tool_direction_error_rad = backwardAttempt.tool_direction_error_rad;
        failure.tool_direction_error_deg = backwardAttempt.tool_direction_error_deg;
        failure.within_joint_limits = backwardAttempt.within_joint_limits;
        failure.joint_limit_margin = backwardAttempt.joint_limit_margin;
        failure.reason = classify_failed_attempt(backwardAttempt);
    end
    failure.all_deterministic_attempt_provenances = {allAttempts.seed_provenance};
    failure.best_position_error_m = minimum_finite([allAttempts.position_error_m]);
    failure.best_direction_error_rad = minimum_finite([allAttempts.tool_direction_error_rad]);
    failure.best_direction_error_deg = rad2deg(failure.best_direction_error_rad);
end


function summary = summarize_result_set(segments, failures)
    points = flatten_successes(segments);
    summary = struct('success_point_count', numel(points), ...
        'failure_count', numel(failures), ...
        'ik_success_segment_count', numel(segments));
    summary.ik_success_rate = summary.success_point_count / ...
        (summary.success_point_count + summary.failure_count);
    if isempty(failures)
        summary.failure_source_indices = zeros(1,0);
        summary.failure_reasons = {};
    else
        summary.failure_source_indices = [failures.source_index];
        summary.failure_reasons = {failures.reason};
    end
    summary.failure_reason_distribution = label_distribution(summary.failure_reasons);
    if isempty(points)
        summary.overall_max_joint_step = NaN;
        summary.minimum_joint_limit_margin = NaN;
        summary.mean_fk_position_error_m = NaN;
        summary.max_fk_position_error_m = NaN;
        summary.mean_tool_direction_error_deg = NaN;
        summary.max_tool_direction_error_deg = NaN;
        summary.accepted_seed_provenance_distribution = label_distribution({});
        return;
    end
    summary.overall_max_joint_step = maximum_finite(abs(vertcat(points.delta_q)));
    summary.minimum_joint_limit_margin = min(vertcat(points.joint_limit_margin), [], 'all');
    summary.mean_fk_position_error_m = mean([points.position_error_m]);
    summary.max_fk_position_error_m = max([points.position_error_m]);
    summary.mean_tool_direction_error_deg = mean([points.tool_direction_error_deg]);
    summary.max_tool_direction_error_deg = max([points.tool_direction_error_deg]);
    summary.accepted_seed_provenance_distribution = ...
        label_distribution({points.accepted_seed_provenance});
end


function value = minimum_finite(values)
    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = min(values); end
end


function value = maximum_finite(values)
    values = values(isfinite(values));
    if isempty(values), value = NaN; else, value = max(values); end
end


function distribution = label_distribution(labels)
    distribution = struct('labels', {{}}, 'counts', zeros(1,0));
    if isempty(labels), return; end
    [labels, ~, groups] = unique(labels, 'stable');
    distribution.labels = labels;
    distribution.counts = accumarray(groups(:), 1)';
end


function points = flatten_successes(segments)
    points = struct([]);
    for index = 1:numel(segments)
        if isempty(points), points = segments(index).results;
        else, points = [points, segments(index).results]; end %#ok<AGROW>
    end
end


function array = append_point(array, point)
    if isempty(array), array = point; else, array(end + 1) = point; end %#ok<AGROW>
end


function segment = make_segment(index, points)
    segment = struct('preik_segment_index', index, ...
        'source_indices', [points.source_index], 't_ms', [points.t_ms], ...
        'q_rad', vertcat(points.q), 'results', points);
end


function segment = empty_segment()
    segment = struct('preik_segment_index', [], 'source_indices', [], ...
        't_ms', [], 'q_rad', [], 'results', struct([]));
end


function record = empty_rescue_record()
    record = struct('preik_segment_index', NaN, 'source_index', NaN, ...
        'next_success_source_index', NaN, 'next_success_q', [], ...
        'provenance', '', 'attempt', struct([]), 'accepted', false);
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


function reason = classify_failed_attempt(attempt)
    if startsWith(attempt.solver_status, 'exception:'), reason = 'solver_or_fk_exception';
    elseif ~attempt.solver_status_is_success, reason = 'solver_status_not_success';
    elseif ~attempt.within_joint_limits, reason = 'joint_limit_violation';
    elseif ~attempt.within_position_tolerance, reason = 'position_error_exceeds_tolerance';
    elseif ~attempt.within_aiming_tolerance, reason = 'tool_direction_error_exceeds_tolerance';
    else, reason = 'unknown_acceptance_failure'; end
end


function gik = configured_gik(robot, algorithm, allowRandomRestart)
    gik = generalizedInverseKinematics('RigidBodyTree', robot, ...
        'ConstraintInputs', {'position','aiming'}, 'SolverAlgorithm', algorithm);
    parameters = gik.SolverParameters;
    parameters.AllowRandomRestart = allowRandomRestart;
    gik.SolverParameters = parameters;
end


function q = row_configuration(configuration)
    q = reshape(configuration, 1, []);
    if ~isequal(size(q), [1,5]) || any(~isfinite(q))
        error('MonoTeach:InvalidBackwardWritingConfiguration', ...
            'Backward rescue configurations must be finite 1-by-5 rows.');
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
        error('MonoTeach:DegenerateBackwardWritingToolDirection', ...
            'FK body5 local-Z direction must be finite and nonzero.');
    end
    value = vector / magnitude;
end


function value = angle_between_unit_vectors(first, second)
    value = acos(max(-1.0, min(1.0, dot(first, second))));
end


function validate_inputs(robot, segmentSet, forward, taskPlane, posture, candidate)
    if ~isa(robot, 'rigidBodyTree') || ~strcmp(robot.DataFormat, 'row') || ...
            robot.NumBodies ~= 5 || ~isstruct(segmentSet) || ...
            ~isstruct(forward) || ~isfield(forward, 'failures') || ...
            ~isstruct(taskPlane) || ~isstruct(posture) || ...
            candidate.normal_sign ~= 1 || posture.allow_random_restart || ...
            ~strcmp(string(posture.solver_algorithm), "BFGSGradientProjection")
        error('MonoTeach:InvalidBackwardWritingRescueInput', ...
            'Backward rescue requires frozen deterministic +normal BFGS inputs.');
    end
end
