function continuousTrajectory = generate_minimum_jerk_continuous_joint_trajectory( ...
        timedTrajectory, timingConfig)
%GENERATE_MINIMUM_JERK_CONTINUOUS_JOINT_TRAJECTORY Derive continuous q(t).
%
% Each TimedJointTrajectory IK-success segment is independently interpolated
% with minjerkpolytraj.  It receives zero velocity and acceleration only at
% its two ends.  Middle waypoint boundary conditions remain NaN, so MATLAB's
% minimum-jerk optimization determines them.  No failure-separated segments
% are joined, and the input artifact is read-only.

    if nargin < 2 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config();
    end
    validate_inputs(timedTrajectory, timingConfig);

    continuousSegments = repmat(empty_segment(), 1, 0);
    for inputIndex = 1:numel(timedTrajectory.segments)
        continuousSegments(end + 1) = generate_one_segment( ...
            timedTrajectory.segments(inputIndex), timingConfig); %#ok<AGROW>
    end

    continuousTrajectory = struct();
    continuousTrajectory.artifact_type = 'ContinuousJointTrajectory';
    continuousTrajectory.generation_method = 'minimum_jerk_polytraj_v1';
    continuousTrajectory.coordinate_space = 'joint';
    continuousTrajectory.units = 'rad';
    continuousTrajectory.time_reference = 'local_per_ik_success_segment';
    continuousTrajectory.time_allocation_optimized = false;
    continuousTrajectory.timing_config = timingConfig;
    continuousTrajectory.segments = continuousSegments;
    continuousTrajectory.summary = summarize(continuousSegments, timingConfig);
end


function segment = generate_one_segment(timedSegment, config)

    initialTimes = timedSegment.t_s(:)';
    waypointQ = timedSegment.q_rad;
    waypointCount = size(waypointQ, 1);
    if waypointCount == 1
        segment = stationary_segment(timedSegment, config);
        return;
    end

    currentTimes = initialTimes;
    initialDuration = initialTimes(end) - initialTimes(1);
    initialVelocityRatio = NaN;
    initialAccelerationRatio = NaN;
    for iteration = 0:config.max_time_stretch_iterations
        guarded = generate_guarded_minimum_jerk_profile( ...
            waypointQ, currentTimes, config);
        generated = guarded.generated;
        [withinLimits, velocityRatio, accelerationRatio] = ...
            limit_evidence(generated, config);
        if iteration == 0
            initialVelocityRatio = velocityRatio;
            initialAccelerationRatio = accelerationRatio;
        end
        if withinLimits
            segment = assemble_segment(timedSegment, generated, initialTimes, ...
                currentTimes, initialDuration, iteration, true, ...
                initialVelocityRatio, initialAccelerationRatio, guarded, config);
            return;
        end
        if iteration == config.max_time_stretch_iterations
            error('MonoTeach:MinimumJerkTimeStretchLimitReached', ...
                ['Segment %d remains outside velocity/acceleration limits ' ...
                'after %d deterministic time-stretch iterations.'], ...
                timedSegment.segment_index, config.max_time_stretch_iterations);
        end
        scale = max([1.0, velocityRatio, sqrt(accelerationRatio)]);
        % A tiny deterministic margin avoids repeated iterations from roundoff
        % when a peak lies exactly on a limit after analytical time scaling.
        currentTimes = currentTimes(1) + ...
            (currentTimes - currentTimes(1)) * (scale * (1.0 + 1.0e-10));
    end
end


function [withinLimits, velocityRatio, accelerationRatio] = limit_evidence(generated, config)

    peakVelocity = max(abs(generated.qd_rad_s), [], 1);
    peakAcceleration = max(abs(generated.qdd_rad_s2), [], 1);
    velocityRatio = max(peakVelocity ./ config.max_velocity_rad_s);
    accelerationRatio = max(peakAcceleration ./ config.max_acceleration_rad_s2);
    withinLimits = velocityRatio <= 1.0 + limit_epsilon() && ...
        accelerationRatio <= 1.0 + limit_epsilon();
end


function segment = assemble_segment(timedSegment, generated, initialTimes, ...
        finalTimes, initialDuration, iteration, limitsSatisfied, ...
        initialVelocityRatio, initialAccelerationRatio, guarded, config)

    segment = empty_segment();
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
    segment.velocity_boundary_condition_rad_s = ...
        guarded.velocity_boundary_condition_rad_s;
    segment.acceleration_boundary_condition_rad_s2 = ...
        guarded.acceleration_boundary_condition_rad_s2;
    segment.overshoot_before_patch = guarded.overshoot_before_patch;
    segment.overshoot_after_patch = guarded.overshoot_after_patch;
    segment.overshoot_patch_iterations = guarded.overshoot_patch_iterations;
    segment.overshoot_patch_selected_iteration = ...
        guarded.overshoot_patch_selected_iteration;
    segment.overshoot_patch_applied = guarded.overshoot_patch_applied;
    segment.velocity_boundary_modified_mask = ...
        guarded.velocity_boundary_modified_mask;
    segment.acceleration_boundary_modified_mask = ...
        guarded.acceleration_boundary_modified_mask;
    segment.velocity_boundary_zeroed_at_reversal_mask = ...
        guarded.velocity_boundary_zeroed_at_reversal_mask;
    segment.t_s = generated.sample_t_s;
    segment.q_rad = generated.q_rad;
    segment.qd_rad_s = generated.qd_rad_s;
    segment.qdd_rad_s2 = generated.qdd_rad_s2;
    segment.qddd_rad_s3 = generated.qddd_rad_s3;
    segment.unpatched_q_rad = guarded.plain_generated.q_rad;
    segment.unpatched_qd_rad_s = guarded.plain_generated.qd_rad_s;
    segment.unpatched_qdd_rad_s2 = guarded.plain_generated.qdd_rad_s2;
    segment.unpatched_qddd_rad_s3 = guarded.plain_generated.qddd_rad_s3;
    segment.unpatched_piecewise_polynomial = guarded.plain_generated.pp;
    segment.piecewise_polynomial = generated.pp;
    segment.initial_duration_s = initialDuration;
    segment.duration_s = finalTimes(end) - finalTimes(1);
    segment.time_stretch_iterations = iteration;
    segment.time_stretch_factor = segment.duration_s / initialDuration;
    segment.was_time_stretched = iteration > 0;
    segment.initial_velocity_limit_ratio = initialVelocityRatio;
    segment.initial_acceleration_limit_ratio = initialAccelerationRatio;
    segment.max_observed_abs_velocity_rad_s = max(abs(segment.qd_rad_s), [], 1);
    segment.max_observed_abs_acceleration_rad_s2 = max(abs(segment.qdd_rad_s2), [], 1);
    segment.max_observed_abs_jerk_rad_s3 = max(abs(segment.qddd_rad_s3), [], 1);
    segment.velocity_limits_satisfied = all( ...
        segment.max_observed_abs_velocity_rad_s <= ...
        config.max_velocity_rad_s + limit_epsilon());
    segment.acceleration_limits_satisfied = all( ...
        segment.max_observed_abs_acceleration_rad_s2 <= ...
        config.max_acceleration_rad_s2 + limit_epsilon());
    segment.limits_satisfied = limitsSatisfied && ...
        segment.velocity_limits_satisfied && segment.acceleration_limits_satisfied;
end


function segment = stationary_segment(timedSegment, config)

    segment = empty_segment();
    segment.segment_index = timedSegment.segment_index;
    segment.ik_success_segment_index = timedSegment.ik_success_segment_index;
    segment.preik_segment_index = timedSegment.preik_segment_index;
    segment.source_indices = timedSegment.source_indices;
    segment.waypoint_q_rad = timedSegment.q_rad;
    segment.initial_waypoint_t_s = timedSegment.t_s;
    segment.waypoint_t_s = timedSegment.t_s;
    segment.waypoint_indices = 1;
    segment.waypoint_provenance = {timedSegment.waypoints.provenance}';
    segment.velocity_boundary_condition_rad_s = 0.0 * timedSegment.q_rad;
    segment.acceleration_boundary_condition_rad_s2 = 0.0 * timedSegment.q_rad;
    segment.overshoot_before_patch = empty_overshoot_evidence(config);
    segment.overshoot_after_patch = empty_overshoot_evidence(config);
    segment.overshoot_patch_iterations = 0;
    segment.overshoot_patch_selected_iteration = 0;
    segment.overshoot_patch_applied = false;
    segment.velocity_boundary_modified_mask = false(1, 5);
    segment.acceleration_boundary_modified_mask = false(1, 5);
    segment.velocity_boundary_zeroed_at_reversal_mask = false(1, 5);
    segment.t_s = timedSegment.t_s;
    segment.q_rad = timedSegment.q_rad;
    segment.qd_rad_s = zeros(1, 5);
    segment.qdd_rad_s2 = zeros(1, 5);
    segment.qddd_rad_s3 = zeros(1, 5);
    segment.unpatched_q_rad = timedSegment.q_rad;
    segment.unpatched_qd_rad_s = zeros(1, 5);
    segment.unpatched_qdd_rad_s2 = zeros(1, 5);
    segment.unpatched_qddd_rad_s3 = zeros(1, 5);
    segment.unpatched_piecewise_polynomial = struct();
    segment.piecewise_polynomial = struct();
    segment.initial_duration_s = 0.0;
    segment.duration_s = 0.0;
    segment.time_stretch_iterations = 0;
    segment.time_stretch_factor = 1.0;
    segment.was_time_stretched = false;
    segment.initial_velocity_limit_ratio = 0.0;
    segment.initial_acceleration_limit_ratio = 0.0;
    segment.max_observed_abs_velocity_rad_s = zeros(1, 5);
    segment.max_observed_abs_acceleration_rad_s2 = zeros(1, 5);
    segment.max_observed_abs_jerk_rad_s3 = zeros(1, 5);
    segment.velocity_limits_satisfied = true;
    segment.acceleration_limits_satisfied = true;
    segment.limits_satisfied = true;
    %#ok<NASGU>
    config = config;
end


function summary = summarize(segments, config)

    summary = struct();
    summary.continuous_segment_count = numel(segments);
    summary.continuous_sample_count = sum(arrayfun( ...
        @(segment) numel(segment.t_s), segments));
    summary.total_independent_duration_s = sum([segments.duration_s]);
    summary.max_velocity_rad_s = config.max_velocity_rad_s;
    summary.max_acceleration_rad_s2 = config.max_acceleration_rad_s2;
    summary.time_allocation_optimized = false;
    if isempty(segments)
        summary.max_observed_abs_velocity_rad_s = NaN(1, 5);
        summary.max_observed_abs_acceleration_rad_s2 = NaN(1, 5);
        summary.max_observed_abs_jerk_rad_s3 = NaN(1, 5);
        summary.all_limits_satisfied = true;
        return;
    end
    summary.max_observed_abs_velocity_rad_s = max(vertcat( ...
        segments.max_observed_abs_velocity_rad_s), [], 1);
    summary.max_observed_abs_acceleration_rad_s2 = max(vertcat( ...
        segments.max_observed_abs_acceleration_rad_s2), [], 1);
    summary.max_observed_abs_jerk_rad_s3 = max(vertcat( ...
        segments.max_observed_abs_jerk_rad_s3), [], 1);
    summary.all_limits_satisfied = all([segments.limits_satisfied]);
end


function validate_inputs(timedTrajectory, config)

    requiredConfig = {'max_velocity_rad_s', 'max_acceleration_rad_s2', ...
        'continuous_sample_period_s', 'max_time_stretch_iterations', ...
        'overshoot_numerical_epsilon_rad', 'overshoot_guard_sample_period_s', ...
        'max_overshoot_correction_iterations', ...
        'overshoot_boundary_damping_factor'};
    if ~isstruct(config) || ~all(isfield(config, requiredConfig)) || ...
            ~isequal(size(config.max_velocity_rad_s), [1, 5]) || ...
            ~isequal(size(config.max_acceleration_rad_s2), [1, 5]) || ...
            any(~isfinite(config.max_velocity_rad_s)) || ...
            any(~isfinite(config.max_acceleration_rad_s2)) || ...
            any(config.max_velocity_rad_s <= 0) || ...
            any(config.max_acceleration_rad_s2 <= 0) || ...
            ~isscalar(config.continuous_sample_period_s) || ...
            ~isfinite(config.continuous_sample_period_s) || ...
            config.continuous_sample_period_s <= 0 || ...
            ~isscalar(config.overshoot_numerical_epsilon_rad) || ...
            ~isfinite(config.overshoot_numerical_epsilon_rad) || ...
            config.overshoot_numerical_epsilon_rad < 0 || ...
            ~isscalar(config.overshoot_guard_sample_period_s) || ...
            ~isfinite(config.overshoot_guard_sample_period_s) || ...
            config.overshoot_guard_sample_period_s <= 0 || ...
            ~isscalar(config.max_time_stretch_iterations) || ...
            config.max_time_stretch_iterations < 0 || ...
            config.max_time_stretch_iterations ~= floor(config.max_time_stretch_iterations)
        error('MonoTeach:InvalidMinimumJerkTrajectoryConfig', ...
            'Config requires finite positive velocity/acceleration limits, sample period, and integer iteration cap.');
    end
    if ~isstruct(timedTrajectory) || ~isscalar(timedTrajectory) || ...
            ~isfield(timedTrajectory, 'artifact_type') || ...
            ~strcmp(string(timedTrajectory.artifact_type), "TimedJointTrajectory") || ...
            ~isfield(timedTrajectory, 'segments') || ...
            ~isstruct(timedTrajectory.segments)
        error('MonoTeach:InvalidTimedJointTrajectory', ...
            'Input must be a TimedJointTrajectory with timed success segments.');
    end
    for index = 1:numel(timedTrajectory.segments)
        segment = timedTrajectory.segments(index);
        requiredSegment = {'segment_index', 'ik_success_segment_index', ...
            'preik_segment_index', 'source_indices', 'waypoints', 'q_rad', 't_s'};
        if ~all(isfield(segment, requiredSegment)) || ...
                ~isequal(size(segment.q_rad, 2), 5) || ...
                size(segment.q_rad, 1) < 1 || ...
                numel(segment.t_s) ~= size(segment.q_rad, 1) || ...
                any(~isfinite(segment.q_rad), 'all') || ...
                any(~isfinite(segment.t_s)) || ...
                any(diff(segment.t_s(:)) <= 0)
            error('MonoTeach:InvalidTimedJointSegment', ...
                'Timed segment %d needs finite 5-DOF q and strictly increasing t.', index);
        end
    end
end


function epsilon = limit_epsilon()

    epsilon = 1.0e-10;
end


function evidence = empty_overshoot_evidence(config)

    evidence = struct('numerical_epsilon_rad', ...
        config.overshoot_numerical_epsilon_rad, 'records', struct([]), ...
        'count', 0, 'has_overshoot', false, ...
        'max_overshoot_magnitude_rad', 0.0);
end


function segment = empty_segment()

    segment = struct('segment_index', NaN, 'ik_success_segment_index', NaN, ...
        'preik_segment_index', NaN, 'source_indices', zeros(1, 0), ...
        'waypoint_q_rad', zeros(0, 5), 'initial_waypoint_t_s', zeros(0, 1), ...
        'waypoint_t_s', zeros(0, 1), 'waypoint_indices', zeros(0, 1), ...
        'waypoint_provenance', {cell(0, 1)}, ...
        'waypoint_qd_rad_s', zeros(0, 5), ...
        'waypoint_qdd_rad_s2', zeros(0, 5), ...
        'velocity_boundary_condition_rad_s', zeros(0, 5), ...
        'acceleration_boundary_condition_rad_s2', zeros(0, 5), ...
        'overshoot_before_patch', struct(), 'overshoot_after_patch', struct(), ...
        'overshoot_patch_iterations', NaN, ...
        'overshoot_patch_selected_iteration', NaN, ...
        'overshoot_patch_applied', false, ...
        'velocity_boundary_modified_mask', false(0, 5), ...
        'acceleration_boundary_modified_mask', false(0, 5), ...
        'velocity_boundary_zeroed_at_reversal_mask', false(0, 5), ...
        't_s', zeros(0, 1), 'q_rad', zeros(0, 5), ...
        'qd_rad_s', zeros(0, 5), 'qdd_rad_s2', zeros(0, 5), ...
        'qddd_rad_s3', zeros(0, 5), 'piecewise_polynomial', struct(), ...
        'unpatched_q_rad', zeros(0, 5), ...
        'unpatched_qd_rad_s', zeros(0, 5), ...
        'unpatched_qdd_rad_s2', zeros(0, 5), ...
        'unpatched_qddd_rad_s3', zeros(0, 5), ...
        'unpatched_piecewise_polynomial', struct(), ...
        'initial_duration_s', NaN, 'duration_s', NaN, ...
        'time_stretch_iterations', NaN, 'time_stretch_factor', NaN, ...
        'was_time_stretched', false, ...
        'initial_velocity_limit_ratio', NaN, ...
        'initial_acceleration_limit_ratio', NaN, ...
        'max_observed_abs_velocity_rad_s', NaN(1, 5), ...
        'max_observed_abs_acceleration_rad_s2', NaN(1, 5), ...
        'max_observed_abs_jerk_rad_s3', NaN(1, 5), ...
        'velocity_limits_satisfied', false, ...
        'acceleration_limits_satisfied', false, 'limits_satisfied', false);
end
