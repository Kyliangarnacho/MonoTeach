function baselines = generate_polynomial_waypoint_diagnostic_baselines( ...
        referenceContinuousTrajectory, timingConfig)
%GENERATE_POLYNOMIAL_WAYPOINT_DIAGNOSTIC_BASELINES Fixed-time cubic/quintic.
%
% Diagnostic-only baselines using exactly the patched minimum-jerk waypoint
% positions, final waypoint times, and output sample times.  They never run
% IK, time allocation, overshoot damping, or limit-enforcing time stretching.

    if nargin < 2 || isempty(timingConfig)
        timingConfig = referenceContinuousTrajectory.timing_config;
    end
    validate_inputs(referenceContinuousTrajectory, timingConfig);
    cubicSegments = repmat(empty_segment(), 1, 0);
    quinticSegments = repmat(empty_segment(), 1, 0);
    for index = 1:numel(referenceContinuousTrajectory.segments)
        source = referenceContinuousTrajectory.segments(index);
        cubicSegments(end + 1) = cubic_segment(source, timingConfig); %#ok<AGROW>
        quinticSegments(end + 1) = quintic_segment(source, timingConfig); %#ok<AGROW>
    end
    baselines = struct();
    baselines.cubic = trajectory_contract('plain_cubic_spline_diagnostic', ...
        cubicSegments, timingConfig);
    baselines.quintic = trajectory_contract('plain_quintic_hermite_diagnostic', ...
        quinticSegments, timingConfig);
end


function trajectory = trajectory_contract(method, segments, config)

    trajectory = struct('artifact_type', 'ContinuousJointTrajectory', ...
        'generation_method', method, 'coordinate_space', 'joint', ...
        'units', 'rad', 'time_reference', 'local_per_ik_success_segment', ...
        'time_allocation_optimized', false, 'timing_config', config, ...
        'segments', segments);
end


function segment = cubic_segment(source, config)

    times = source.waypoint_t_s(:)';
    samples = source.t_s(:)';
    waypointQ = source.waypoint_q_rad;
    pp = spline(times, waypointQ');
    segment = assemble(source, pp, samples, times, config, ...
        'plain_cubic_spline_diagnostic', [], []);
end


function segment = quintic_segment(source, config)

    times = source.waypoint_t_s(:)';
    samples = source.t_s(:)';
    waypointQ = source.waypoint_q_rad;
    [velocityBC, accelerationBC] = finite_difference_boundaries(waypointQ, times);
    [~, ~, ~, pp] = quinticpolytraj(waypointQ', times, samples, ...
        VelocityBoundaryCondition=velocityBC', ...
        AccelerationBoundaryCondition=accelerationBC');
    segment = assemble(source, pp, samples, times, config, ...
        'plain_quintic_hermite_diagnostic', velocityBC, accelerationBC);
end


function [velocity, acceleration] = finite_difference_boundaries(waypointQ, times)

    count = size(waypointQ, 1);
    velocity = zeros(count, 5);
    acceleration = zeros(count, 5);
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
    % First/last velocity and acceleration stay explicitly zero.
end


function segment = assemble(source, pp, samples, waypointTimes, config, method, ...
        velocityBC, accelerationBC)

    guardTimes = sample_times(waypointTimes, config.overshoot_guard_sample_period_s);
    guardQ = ppval(pp, guardTimes)';
    segment = empty_segment();
    segment.generation_method = method;
    segment.segment_index = source.segment_index;
    segment.ik_success_segment_index = source.ik_success_segment_index;
    segment.preik_segment_index = source.preik_segment_index;
    segment.source_indices = source.source_indices;
    segment.waypoint_q_rad = source.waypoint_q_rad;
    segment.initial_waypoint_t_s = source.initial_waypoint_t_s;
    segment.waypoint_t_s = source.waypoint_t_s;
    segment.waypoint_indices = source.waypoint_indices;
    segment.waypoint_provenance = source.waypoint_provenance;
    segment.t_s = samples(:);
    segment.q_rad = ppval(pp, samples)';
    segment.qd_rad_s = ppval(fnder(pp, 1), samples)';
    segment.qdd_rad_s2 = ppval(fnder(pp, 2), samples)';
    segment.qddd_rad_s3 = ppval(fnder(pp, 3), samples)';
    segment.piecewise_polynomial = pp;
    segment.velocity_boundary_condition_rad_s = velocityBC;
    segment.acceleration_boundary_condition_rad_s2 = accelerationBC;
    segment.overshoot_evidence = detect_joint_interval_overshoot( ...
        guardTimes, guardQ, waypointTimes(:), source.waypoint_q_rad, ...
        config.overshoot_numerical_epsilon_rad);
    segment.max_observed_abs_velocity_rad_s = max(abs(segment.qd_rad_s), [], 1);
    segment.max_observed_abs_acceleration_rad_s2 = max(abs(segment.qdd_rad_s2), [], 1);
end


function times = sample_times(waypointTimes, period)

    times = unique([waypointTimes(1):period:waypointTimes(end), ...
        waypointTimes, waypointTimes(end)]);
end


function validate_inputs(trajectory, config)

    if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
            ~isfield(trajectory, 'artifact_type') || ...
            ~strcmp(string(trajectory.artifact_type), "ContinuousJointTrajectory") || ...
            ~isfield(trajectory, 'segments') || ~isstruct(trajectory.segments) || ...
            ~isstruct(config) || ~isfield(config, 'overshoot_guard_sample_period_s') || ...
            ~isfield(config, 'overshoot_numerical_epsilon_rad')
        error('MonoTeach:InvalidPolynomialDiagnosticInput', ...
            'Diagnostic baselines require a ContinuousJointTrajectory and guard config.');
    end
    for index = 1:numel(trajectory.segments)
        segment = trajectory.segments(index);
        required = {'segment_index', 'ik_success_segment_index', ...
            'preik_segment_index', 'source_indices', 'waypoint_q_rad', ...
            'initial_waypoint_t_s', 'waypoint_t_s', 'waypoint_indices', ...
            'waypoint_provenance', 't_s'};
        if ~all(isfield(segment, required)) || ...
                size(segment.waypoint_q_rad, 1) < 2 || ...
                ~isequal(size(segment.waypoint_q_rad, 2), 5) || ...
                numel(segment.waypoint_t_s) ~= size(segment.waypoint_q_rad, 1) || ...
                any(diff(segment.waypoint_t_s(:)) <= 0) || ...
                any(diff(segment.t_s(:)) <= 0)
            error('MonoTeach:InvalidPolynomialDiagnosticSegment', ...
                'Each diagnostic segment needs >=2 waypoints and fixed increasing times.');
        end
    end
end


function segment = empty_segment()

    segment = struct('generation_method', '', 'segment_index', NaN, ...
        'ik_success_segment_index', NaN, 'preik_segment_index', NaN, ...
        'source_indices', zeros(1, 0), 'waypoint_q_rad', zeros(0, 5), ...
        'initial_waypoint_t_s', zeros(0, 1), 'waypoint_t_s', zeros(0, 1), ...
        'waypoint_indices', zeros(0, 1), 'waypoint_provenance', {cell(0, 1)}, ...
        't_s', zeros(0, 1), 'q_rad', zeros(0, 5), ...
        'qd_rad_s', zeros(0, 5), 'qdd_rad_s2', zeros(0, 5), ...
        'qddd_rad_s3', zeros(0, 5), 'piecewise_polynomial', struct(), ...
        'velocity_boundary_condition_rad_s', [], ...
        'acceleration_boundary_condition_rad_s2', [], ...
        'overshoot_evidence', struct(), ...
        'max_observed_abs_velocity_rad_s', NaN(1, 5), ...
        'max_observed_abs_acceleration_rad_s2', NaN(1, 5));
end
