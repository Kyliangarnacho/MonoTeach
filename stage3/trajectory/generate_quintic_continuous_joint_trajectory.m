function continuousTrajectory = generate_quintic_continuous_joint_trajectory( ...
        timedTrajectory, timingConfig, robotContext)
%GENERATE_QUINTIC_CONTINUOUS_JOINT_TRAJECTORY Formal quintic execution V1.
%
% Derives an independent ContinuousJointTrajectory for every timed IK-success
% segment.  It preserves the position waypoints exactly, uses deterministic
% finite-difference interior velocity/acceleration boundary conditions, and
% gives only the two endpoints zero velocity and acceleration.  If sampled
% velocity or acceleration exceeds the planning limits, the entire segment's
% local time axis is uniformly stretched and the finite differences are
% recomputed.  The input TimedJointTrajectory is never modified.

    if nargin < 3 || isempty(robotContext)
        robotContext = load_robot_context("legacy5");
    end
    if nargin < 2 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config(robotContext);
    end
    validate_inputs(timedTrajectory, timingConfig, robotContext);
    inputSnapshot = timedTrajectory;
    limits = robotContext.joint_limits;
    segments = repmat(empty_segment(robotContext.dof), 1, 0);
    for index = 1:numel(timedTrajectory.segments)
        segments(end + 1) = generate_one_segment( ...
            timedTrajectory.segments(index), timingConfig, limits, robotContext.dof); %#ok<AGROW>
    end
    if ~isequaln(timedTrajectory, inputSnapshot)
        error('MonoTeach:QuinticTimedInputMutation', ...
            'Quintic generation must not modify the TimedJointTrajectory input.');
    end

    continuousTrajectory = struct();
    continuousTrajectory.artifact_type = 'ContinuousJointTrajectory';
    continuousTrajectory.generation_method = 'quintic_hermite_v1';
    continuousTrajectory.coordinate_space = 'joint';
    continuousTrajectory.units = 'rad';
    continuousTrajectory.time_reference = 'local_per_ik_success_segment';
    continuousTrajectory.time_allocation_optimized = false;
    continuousTrajectory.robot_id = robotContext.id;
    continuousTrajectory.robot_backend = robotContext.backend;
    continuousTrajectory.timing_config = timingConfig;
    continuousTrajectory.segments = segments;
    continuousTrajectory.summary = summarize(segments, timingConfig);
end


function segment = generate_one_segment(timedSegment, config, limits, dof)

    initialTimes = timedSegment.t_s(:)';
    waypointQ = timedSegment.q_rad;
    if size(waypointQ, 1) == 1
        segment = stationary_segment(timedSegment, config, limits, dof);
        return;
    end
    initialDuration = initialTimes(end) - initialTimes(1);
    currentTimes = initialTimes;
    initialVelocityRatio = NaN;
    initialAccelerationRatio = NaN;
    initialPeakVelocity = NaN(1, dof);
    initialPeakAcceleration = NaN(1, dof);
    for iteration = 0:config.max_time_stretch_iterations
        generated = generate_profile(waypointQ, currentTimes, config);
        evidence = safety_evidence(generated, config, limits);
        if iteration == 0
            initialVelocityRatio = evidence.velocity_ratio;
            initialAccelerationRatio = evidence.acceleration_ratio;
            initialPeakVelocity = evidence.peak_velocity_rad_s;
            initialPeakAcceleration = evidence.peak_acceleration_rad_s2;
        end
        if evidence.velocity_limits_satisfied && evidence.acceleration_limits_satisfied
            segment = assemble_segment(timedSegment, generated, initialTimes, ...
                currentTimes, initialDuration, iteration, evidence, ...
                initialVelocityRatio, initialAccelerationRatio, ...
                initialPeakVelocity, initialPeakAcceleration);
            return;
        end
        if iteration == config.max_time_stretch_iterations
            error('MonoTeach:QuinticTimeStretchLimitReached', ...
                ['Segment %d remains outside velocity/acceleration limits ' ...
                'after %d deterministic time-stretch iterations.'], ...
                timedSegment.segment_index, config.max_time_stretch_iterations);
        end
        scale = max([1.0, evidence.velocity_ratio, ...
            sqrt(evidence.acceleration_ratio)]);
        currentTimes = currentTimes(1) + (currentTimes - currentTimes(1)) * ...
            (scale * (1.0 + 1.0e-10));
    end
end


function generated = generate_profile(waypointQ, waypointTimes, config)

    [velocityBC, accelerationBC] = finite_difference_boundaries( ...
        waypointQ, waypointTimes);
    sampleTimes = trajectory_sample_times(waypointTimes, ...
        config.continuous_sample_period_s);
    [~, ~, ~, pp] = quinticpolytraj(waypointQ', waypointTimes, sampleTimes, ...
        VelocityBoundaryCondition=velocityBC', ...
        AccelerationBoundaryCondition=accelerationBC');
    guardTimes = trajectory_sample_times(waypointTimes, ...
        config.overshoot_guard_sample_period_s);
    generated = struct();
    generated.sample_t_s = sampleTimes(:);
    generated.q_rad = ppval(pp, sampleTimes)';
    generated.qd_rad_s = ppval(fnder(pp, 1), sampleTimes)';
    generated.qdd_rad_s2 = ppval(fnder(pp, 2), sampleTimes)';
    generated.qddd_rad_s3 = ppval(fnder(pp, 3), sampleTimes)';
    generated.waypoint_qd_rad_s = ppval(fnder(pp, 1), waypointTimes)';
    generated.waypoint_qdd_rad_s2 = ppval(fnder(pp, 2), waypointTimes)';
    generated.velocity_boundary_condition_rad_s = velocityBC;
    generated.acceleration_boundary_condition_rad_s2 = accelerationBC;
    generated.piecewise_polynomial = pp;
    generated.overshoot_evidence = detect_joint_interval_overshoot( ...
        guardTimes, ppval(pp, guardTimes)', waypointTimes(:), waypointQ, ...
        config.overshoot_numerical_epsilon_rad);
end


function [velocity, acceleration] = finite_difference_boundaries(waypointQ, times)

    count = size(waypointQ, 1);
    dof = size(waypointQ, 2);
    velocity = zeros(count, dof);
    acceleration = zeros(count, dof);
    for index = 2:count-1
        previousDt = times(index) - times(index - 1);
        nextDt = times(index + 1) - times(index);
        velocity(index, :) = (waypointQ(index + 1, :) - waypointQ(index - 1, :)) / ...
            (times(index + 1) - times(index - 1));
        acceleration(index, :) = 2.0 * ( ...
            (waypointQ(index + 1, :) - waypointQ(index, :)) / nextDt - ...
            (waypointQ(index, :) - waypointQ(index - 1, :)) / previousDt) / ...
            (previousDt + nextDt);
    end
    % Endpoints remain explicitly zero; interiors are inferred, not user input.
end


function evidence = safety_evidence(generated, config, limits)

    evidence = struct();
    evidence.q_finite = all(isfinite(generated.q_rad), 'all');
    evidence.qd_finite = all(isfinite(generated.qd_rad_s), 'all');
    evidence.qdd_finite = all(isfinite(generated.qdd_rad_s2), 'all');
    evidence.qddd_finite = all(isfinite(generated.qddd_rad_s3), 'all');
    evidence.peak_velocity_rad_s = max(abs(generated.qd_rad_s), [], 1);
    evidence.peak_acceleration_rad_s2 = max(abs(generated.qdd_rad_s2), [], 1);
    evidence.velocity_ratio = max(evidence.peak_velocity_rad_s ./ ...
        config.max_velocity_rad_s);
    evidence.acceleration_ratio = max(evidence.peak_acceleration_rad_s2 ./ ...
        config.max_acceleration_rad_s2);
    evidence.velocity_limits_satisfied = evidence.velocity_ratio <= 1.0 + limit_epsilon();
    evidence.acceleration_limits_satisfied = evidence.acceleration_ratio <= 1.0 + limit_epsilon();
    evidence.within_joint_limits = all(generated.q_rad >= limits(:, 1)' - limit_epsilon() & ...
        generated.q_rad <= limits(:, 2)' + limit_epsilon(), 2);
    evidence.joint_limit_margin = min(generated.q_rad - limits(:, 1)', ...
        limits(:, 2)' - generated.q_rad);
    evidence.minimum_joint_limit_margin = min(evidence.joint_limit_margin, [], 'all');
    evidence.joint_limits_satisfied = all(evidence.within_joint_limits);
    evidence.overshoot_evidence = generated.overshoot_evidence;
    evidence.all_finite = evidence.q_finite && evidence.qd_finite && ...
        evidence.qdd_finite && evidence.qddd_finite;
    evidence.all_safety_checks_satisfied = evidence.all_finite && ...
        evidence.velocity_limits_satisfied && evidence.acceleration_limits_satisfied && ...
        evidence.joint_limits_satisfied;
end


function segment = assemble_segment(timedSegment, generated, initialTimes, finalTimes, ...
        initialDuration, iteration, evidence, initialVelocityRatio, initialAccelerationRatio, ...
        initialPeakVelocity, initialPeakAcceleration)

    segment = empty_segment(size(timedSegment.q_rad, 2));
    segment.segment_index = timedSegment.segment_index;
    segment.ik_success_segment_index = timedSegment.ik_success_segment_index;
    segment.preik_segment_index = timedSegment.preik_segment_index;
    segment.source_indices = timedSegment.source_indices;
    segment.waypoint_q_rad = timedSegment.q_rad;
    segment.initial_waypoint_t_s = initialTimes(:);
    segment.waypoint_t_s = finalTimes(:);
    segment.waypoint_indices = (1:numel(finalTimes))';
    segment.waypoint_provenance = {timedSegment.waypoints.provenance}';
    segment.waypoint_qd_rad_s = generated.waypoint_qd_rad_s;
    segment.waypoint_qdd_rad_s2 = generated.waypoint_qdd_rad_s2;
    segment.velocity_boundary_condition_rad_s = generated.velocity_boundary_condition_rad_s;
    segment.acceleration_boundary_condition_rad_s2 = generated.acceleration_boundary_condition_rad_s2;
    segment.t_s = generated.sample_t_s;
    segment.q_rad = generated.q_rad;
    segment.qd_rad_s = generated.qd_rad_s;
    segment.qdd_rad_s2 = generated.qdd_rad_s2;
    segment.qddd_rad_s3 = generated.qddd_rad_s3;
    segment.piecewise_polynomial = generated.piecewise_polynomial;
    segment.overshoot_evidence = evidence.overshoot_evidence;
    segment.initial_duration_s = initialDuration;
    segment.duration_s = finalTimes(end) - finalTimes(1);
    segment.time_stretch_iterations = iteration;
    segment.time_stretch_factor = segment.duration_s / initialDuration;
    segment.was_time_stretched = iteration > 0;
    segment.initial_velocity_limit_ratio = initialVelocityRatio;
    segment.initial_acceleration_limit_ratio = initialAccelerationRatio;
    segment.initial_peak_velocity_rad_s = initialPeakVelocity;
    segment.initial_peak_acceleration_rad_s2 = initialPeakAcceleration;
    segment.max_observed_abs_velocity_rad_s = evidence.peak_velocity_rad_s;
    segment.max_observed_abs_acceleration_rad_s2 = evidence.peak_acceleration_rad_s2;
    segment.max_observed_abs_jerk_rad_s3 = max(abs(segment.qddd_rad_s3), [], 1);
    segment.velocity_limits_satisfied = evidence.velocity_limits_satisfied;
    segment.acceleration_limits_satisfied = evidence.acceleration_limits_satisfied;
    segment.within_joint_limits = evidence.within_joint_limits;
    segment.joint_limit_margin = evidence.joint_limit_margin;
    segment.minimum_joint_limit_margin = evidence.minimum_joint_limit_margin;
    segment.joint_limits_satisfied = evidence.joint_limits_satisfied;
    segment.all_safety_checks_satisfied = evidence.all_safety_checks_satisfied;
end


function segment = stationary_segment(timedSegment, config, limits, dof)

    generated = struct('q_rad', timedSegment.q_rad, 'qd_rad_s', zeros(1, dof), ...
        'qdd_rad_s2', zeros(1, dof), 'qddd_rad_s3', zeros(1, dof), ...
        'overshoot_evidence', empty_overshoot_evidence(config));
    evidence = safety_evidence(generated, config, limits);
    segment = empty_segment(dof);
    segment.segment_index = timedSegment.segment_index;
    segment.ik_success_segment_index = timedSegment.ik_success_segment_index;
    segment.preik_segment_index = timedSegment.preik_segment_index;
    segment.source_indices = timedSegment.source_indices;
    segment.waypoint_q_rad = timedSegment.q_rad;
    segment.initial_waypoint_t_s = timedSegment.t_s;
    segment.waypoint_t_s = timedSegment.t_s;
    segment.waypoint_indices = 1;
    segment.waypoint_provenance = {timedSegment.waypoints.provenance}';
    segment.waypoint_qd_rad_s = zeros(1, dof);
    segment.waypoint_qdd_rad_s2 = zeros(1, dof);
    segment.velocity_boundary_condition_rad_s = zeros(1, dof);
    segment.acceleration_boundary_condition_rad_s2 = zeros(1, dof);
    segment.t_s = timedSegment.t_s;
    segment.q_rad = timedSegment.q_rad;
    segment.qd_rad_s = zeros(1, dof);
    segment.qdd_rad_s2 = zeros(1, dof);
    segment.qddd_rad_s3 = zeros(1, dof);
    segment.overshoot_evidence = evidence.overshoot_evidence;
    segment.initial_duration_s = 0.0;
    segment.duration_s = 0.0;
    segment.time_stretch_iterations = 0;
    segment.time_stretch_factor = 1.0;
    segment.was_time_stretched = false;
    segment.initial_velocity_limit_ratio = 0.0;
    segment.initial_acceleration_limit_ratio = 0.0;
    segment.initial_peak_velocity_rad_s = zeros(1, dof);
    segment.initial_peak_acceleration_rad_s2 = zeros(1, dof);
    segment.max_observed_abs_velocity_rad_s = zeros(1, dof);
    segment.max_observed_abs_acceleration_rad_s2 = zeros(1, dof);
    segment.max_observed_abs_jerk_rad_s3 = zeros(1, dof);
    segment.velocity_limits_satisfied = true;
    segment.acceleration_limits_satisfied = true;
    segment.within_joint_limits = evidence.within_joint_limits;
    segment.joint_limit_margin = evidence.joint_limit_margin;
    segment.minimum_joint_limit_margin = evidence.minimum_joint_limit_margin;
    segment.joint_limits_satisfied = evidence.joint_limits_satisfied;
    segment.all_safety_checks_satisfied = evidence.all_safety_checks_satisfied;
end


function summary = summarize(segments, config)

    summary = struct('continuous_segment_count', numel(segments), ...
        'continuous_sample_count', sum(arrayfun(@(segment) numel(segment.t_s), segments)), ...
        'total_independent_duration_s', sum([segments.duration_s]), ...
        'max_velocity_rad_s', config.max_velocity_rad_s, ...
        'max_acceleration_rad_s2', config.max_acceleration_rad_s2, ...
        'time_allocation_optimized', false);
    if isempty(segments)
        dof = numel(config.max_velocity_rad_s);
        summary.max_observed_abs_velocity_rad_s = NaN(1, dof);
        summary.max_observed_abs_acceleration_rad_s2 = NaN(1, dof);
        summary.max_observed_abs_jerk_rad_s3 = NaN(1, dof);
        summary.all_safety_checks_satisfied = true;
        summary.all_joint_limits_satisfied = true;
        summary.max_joint_overshoot_rad = 0.0;
        return;
    end
    summary.max_observed_abs_velocity_rad_s = max(vertcat( ...
        segments.max_observed_abs_velocity_rad_s), [], 1);
    summary.max_observed_abs_acceleration_rad_s2 = max(vertcat( ...
        segments.max_observed_abs_acceleration_rad_s2), [], 1);
    summary.max_observed_abs_jerk_rad_s3 = max(vertcat( ...
        segments.max_observed_abs_jerk_rad_s3), [], 1);
    summary.all_safety_checks_satisfied = all([segments.all_safety_checks_satisfied]);
    summary.all_joint_limits_satisfied = all([segments.joint_limits_satisfied]);
    summary.max_joint_overshoot_rad = max(arrayfun(@(segment) ...
        segment.overshoot_evidence.max_overshoot_magnitude_rad, segments));
end


function times = trajectory_sample_times(waypointTimes, period)

    times = unique([waypointTimes(1):period:waypointTimes(end), ...
        waypointTimes, waypointTimes(end)]);
end


function evidence = empty_overshoot_evidence(config)

    evidence = struct('numerical_epsilon_rad', ...
        config.overshoot_numerical_epsilon_rad, 'records', struct([]), ...
        'count', 0, 'has_overshoot', false, ...
        'max_overshoot_magnitude_rad', 0.0);
end


function validate_inputs(timedTrajectory, config, robotContext)

    requiredConfig = {'max_velocity_rad_s', 'max_acceleration_rad_s2', ...
        'continuous_sample_period_s', 'overshoot_guard_sample_period_s', ...
        'overshoot_numerical_epsilon_rad', 'max_time_stretch_iterations'};
    if ~isstruct(config) || ~all(isfield(config, requiredConfig)) || ...
            ~isstruct(robotContext) || ~isscalar(robotContext) || ...
            ~all(isfield(robotContext, {'id', 'model', 'dof', 'end_effector', ...
            'joint_limits', 'backend'})) || robotContext.dof < 1 || ...
            ~isequal(size(robotContext.joint_limits), [robotContext.dof, 2]) || ...
            ~isequal(size(config.max_velocity_rad_s), [1, robotContext.dof]) || ...
            ~isequal(size(config.max_acceleration_rad_s2), [1, robotContext.dof]) || ...
            any(~isfinite(config.max_velocity_rad_s)) || ...
            any(~isfinite(config.max_acceleration_rad_s2)) || ...
            any(config.max_velocity_rad_s <= 0) || ...
            any(config.max_acceleration_rad_s2 <= 0) || ...
            ~isscalar(config.continuous_sample_period_s) || ...
            ~isfinite(config.continuous_sample_period_s) || ...
            config.continuous_sample_period_s <= 0 || ...
            ~isscalar(config.overshoot_guard_sample_period_s) || ...
            ~isfinite(config.overshoot_guard_sample_period_s) || ...
            config.overshoot_guard_sample_period_s <= 0 || ...
            ~isscalar(config.overshoot_numerical_epsilon_rad) || ...
            ~isfinite(config.overshoot_numerical_epsilon_rad) || ...
            config.overshoot_numerical_epsilon_rad < 0 || ...
            ~isscalar(config.max_time_stretch_iterations) || ...
            config.max_time_stretch_iterations < 0 || ...
            config.max_time_stretch_iterations ~= floor(config.max_time_stretch_iterations)
        error('MonoTeach:InvalidQuinticTrajectoryConfig', ...
            'Quintic V1 requires finite positive limits, sampling, and an integer stretch cap.');
    end
    if ~isstruct(timedTrajectory) || ~isscalar(timedTrajectory) || ...
            ~isfield(timedTrajectory, 'artifact_type') || ...
            ~strcmp(string(timedTrajectory.artifact_type), "TimedJointTrajectory") || ...
            ~isfield(timedTrajectory, 'segments') || ~isstruct(timedTrajectory.segments)
        error('MonoTeach:InvalidQuinticTimedTrajectory', ...
            'Input must be a TimedJointTrajectory with timed success segments.');
    end
    for index = 1:numel(timedTrajectory.segments)
        segment = timedTrajectory.segments(index);
        requiredSegment = {'segment_index', 'ik_success_segment_index', ...
            'preik_segment_index', 'source_indices', 'waypoints', 'q_rad', 't_s'};
        if ~all(isfield(segment, requiredSegment)) || ...
                ~isequal(size(segment.q_rad, 2), robotContext.dof) || ...
                size(segment.q_rad, 1) < 1 || ...
                numel(segment.t_s) ~= size(segment.q_rad, 1) || ...
                any(~isfinite(segment.q_rad), 'all') || any(~isfinite(segment.t_s)) || ...
                any(diff(segment.t_s(:)) <= 0)
            error('MonoTeach:InvalidQuinticTimedSegment', ...
                'Timed segment %d needs finite context-DOF q and increasing local time.', index);
        end
    end
end


function epsilon = limit_epsilon()

    epsilon = 1.0e-10;
end


function segment = empty_segment(dof)

    segment = struct('segment_index', NaN, 'ik_success_segment_index', NaN, ...
        'preik_segment_index', NaN, 'source_indices', zeros(1, 0), ...
        'waypoint_q_rad', zeros(0, dof), 'initial_waypoint_t_s', zeros(0, 1), ...
        'waypoint_t_s', zeros(0, 1), 'waypoint_indices', zeros(0, 1), ...
        'waypoint_provenance', {cell(0, 1)}, ...
        'waypoint_qd_rad_s', zeros(0, dof), 'waypoint_qdd_rad_s2', zeros(0, dof), ...
        'velocity_boundary_condition_rad_s', zeros(0, dof), ...
        'acceleration_boundary_condition_rad_s2', zeros(0, dof), ...
        't_s', zeros(0, 1), 'q_rad', zeros(0, dof), ...
        'qd_rad_s', zeros(0, dof), 'qdd_rad_s2', zeros(0, dof), ...
        'qddd_rad_s3', zeros(0, dof), 'piecewise_polynomial', struct(), ...
        'overshoot_evidence', struct(), 'initial_duration_s', NaN, ...
        'duration_s', NaN, 'time_stretch_iterations', NaN, ...
        'time_stretch_factor', NaN, 'was_time_stretched', false, ...
        'initial_velocity_limit_ratio', NaN, 'initial_acceleration_limit_ratio', NaN, ...
        'initial_peak_velocity_rad_s', NaN(1, dof), ...
        'initial_peak_acceleration_rad_s2', NaN(1, dof), ...
        'max_observed_abs_velocity_rad_s', NaN(1, dof), ...
        'max_observed_abs_acceleration_rad_s2', NaN(1, dof), ...
        'max_observed_abs_jerk_rad_s3', NaN(1, dof), ...
        'velocity_limits_satisfied', false, 'acceleration_limits_satisfied', false, ...
        'within_joint_limits', false(0, 1), 'joint_limit_margin', zeros(0, dof), ...
        'minimum_joint_limit_margin', NaN, 'joint_limits_satisfied', false, ...
        'all_safety_checks_satisfied', false);
end
