function timedTrajectory = time_parameterize_ik_success_segments( ...
        ikResultSet, timingConfig, robotContext)
%TIME_PARAMETERIZE_IK_SUCCESS_SEGMENTS Derive velocity-limited joint timing.
%
% Every IK-success segment receives an independent local clock.  A failure
% remains a hard barrier: this function never creates a time interval between
% two separate IK-success segments.  The input IKResultSet is read-only.

    if nargin < 3 || isempty(robotContext)
        robotContext = load_robot_context("legacy5");
    end
    if nargin < 2 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config(robotContext);
    end
    validate_inputs(ikResultSet, timingConfig, robotContext);

    timedSegments = repmat(empty_segment(robotContext.dof), 1, 0);
    for ikSegmentIndex = 1:numel(ikResultSet.segments)
        ikSegment = ikResultSet.segments(ikSegmentIndex);
        timedSegments(end + 1) = time_one_segment( ...
            ikSegment, ikSegmentIndex, timingConfig, robotContext.dof); %#ok<AGROW>
    end

    timedTrajectory = struct();
    timedTrajectory.artifact_type = 'TimedJointTrajectory';
    timedTrajectory.coordinate_space = 'joint';
    timedTrajectory.units = 'rad';
    timedTrajectory.timing_method = 'velocity_limited_waypoint_baseline';
    timedTrajectory.time_reference = 'local_per_ik_success_segment';
    timedTrajectory.robot_id = robotContext.id;
    timedTrajectory.robot_backend = robotContext.backend;
    timedTrajectory.timing_config = timingConfig;
    timedTrajectory.input_failure_count = numel(ikResultSet.failures);
    timedTrajectory.segments = timedSegments;
    timedTrajectory.summary = summarize(timedSegments, timingConfig);
end


function segment = time_one_segment(ikSegment, ikSegmentIndex, config, dof)

    results = ikSegment.results;
    waypointCount = numel(results);
    waypoints = repmat(empty_waypoint(dof), 1, waypointCount);
    previousQ = [];
    elapsedS = 0.0;

    for pointIndex = 1:waypointCount
        result = results(pointIndex);
        q = require_q(result, ikSegmentIndex, pointIndex, dof);
        waypoint = empty_waypoint(dof);
        waypoint.q_rad = q;
        waypoint.segment_index = ikSegmentIndex;
        waypoint.ik_success_segment_index = ikSegmentIndex;
        waypoint.preik_segment_index = ikSegment.preik_segment_index;
        waypoint.source_index = result.source_index;
        waypoint.source_t_ms = result.t_ms;
        waypoint.provenance = source_provenance(result, ikSegmentIndex, pointIndex);

        if isempty(previousQ)
            waypoint.delta_q_rad = zeros(1, dof);
            waypoint.dt_s = 0.0;
            waypoint.t_s = 0.0;
            waypoint.step_velocity_rad_s = zeros(1, dof);
            waypoint.within_velocity_limits = true;
        else
            waypoint.delta_q_rad = q - previousQ;
            jointDt = abs(waypoint.delta_q_rad) ./ config.max_velocity_rad_s;
            waypoint.dt_s = max([jointDt, config.minimum_segment_dt_s]);
            elapsedS = elapsedS + waypoint.dt_s;
            waypoint.t_s = elapsedS;
            waypoint.step_velocity_rad_s = waypoint.delta_q_rad ./ waypoint.dt_s;
            waypoint.within_velocity_limits = all( ...
                abs(waypoint.step_velocity_rad_s) <= ...
                config.max_velocity_rad_s + velocity_epsilon());
        end
        waypoints(pointIndex) = waypoint;
        previousQ = q;
    end

    segment = empty_segment(dof);
    segment.segment_index = ikSegmentIndex;
    segment.ik_success_segment_index = ikSegmentIndex;
    segment.preik_segment_index = ikSegment.preik_segment_index;
    segment.source_indices = [waypoints.source_index];
    segment.waypoints = waypoints;
    segment.q_rad = vertcat(waypoints.q_rad);
    segment.t_s = [waypoints.t_s]';
    segment.dt_s = [waypoints.dt_s]';
    segment.step_velocity_rad_s = vertcat(waypoints.step_velocity_rad_s);
    segment.duration_s = waypoints(end).t_s;
    segment.max_abs_step_velocity_rad_s = max(abs( ...
        vertcat(waypoints.step_velocity_rad_s)), [], 1);
end


function summary = summarize(segments, config)

    summary = struct();
    summary.timed_segment_count = numel(segments);
    summary.timed_waypoint_count = sum(arrayfun( ...
        @(segment) numel(segment.waypoints), segments));
    summary.total_independent_duration_s = sum([segments.duration_s]);
    summary.max_velocity_rad_s = config.max_velocity_rad_s;
    if isempty(segments)
        summary.max_observed_abs_velocity_rad_s = NaN(1, numel(config.max_velocity_rad_s));
        summary.all_velocity_limits_satisfied = true;
        return;
    end
    velocityMatrix = zeros(0, numel(config.max_velocity_rad_s));
    allWithinLimits = true;
    for segmentIndex = 1:numel(segments)
        waypoints = segments(segmentIndex).waypoints;
        velocityMatrix = [velocityMatrix; ...
            vertcat(waypoints.step_velocity_rad_s)]; %#ok<AGROW>
        allWithinLimits = allWithinLimits && all( ...
            [waypoints.within_velocity_limits]);
    end
    summary.max_observed_abs_velocity_rad_s = max(abs(velocityMatrix), [], 1);
    summary.all_velocity_limits_satisfied = allWithinLimits;
end


function provenance = source_provenance(result, ikSegmentIndex, pointIndex)

    provenance = struct('ik_success_segment_index', ikSegmentIndex, ...
        'point_index_within_segment', pointIndex, ...
        'source_index', result.source_index, 'source_t_ms', result.t_ms);
    if isfield(result, 'preik_segment_index')
        provenance.preik_segment_index = result.preik_segment_index;
    end
    if isfield(result, 'resampled_provenance')
        provenance.resampled_provenance = result.resampled_provenance;
    end
end


function q = require_q(result, segmentIndex, pointIndex, dof)

    if ~isfield(result, 'q') || ~isnumeric(result.q) || ~isreal(result.q) || ...
            ~isequal(size(result.q), [1, dof]) || any(~isfinite(result.q))
        error('MonoTeach:InvalidTimedJointWaypoint', ...
            'Segment %d point %d must have finite 1-by-context-DOF q.', ...
            segmentIndex, pointIndex);
    end
    q = result.q;
end


function validate_inputs(ikResultSet, config, robotContext)

    if ~isstruct(robotContext) || ~isscalar(robotContext) || ...
            ~all(isfield(robotContext, {'id', 'dof', 'joint_limits', 'backend'})) || ...
            robotContext.dof < 1 || ...
            ~isequal(size(robotContext.joint_limits), [robotContext.dof, 2])
        error('MonoTeach:InvalidTimedRobotContext', ...
            'Timing requires RobotContext DOF and joint limits.');
    end
    validate_ik_result_set(ikResultSet, robotContext.dof);
    required = {'max_velocity_rad_s', 'minimum_segment_dt_s'};
    if ~isstruct(config) || ~all(isfield(config, required)) || ...
            ~isnumeric(config.max_velocity_rad_s) || ...
            ~isreal(config.max_velocity_rad_s) || ...
            ~isequal(size(config.max_velocity_rad_s), [1, robotContext.dof]) || ...
            any(~isfinite(config.max_velocity_rad_s)) || ...
            any(config.max_velocity_rad_s <= 0) || ...
            ~isnumeric(config.minimum_segment_dt_s) || ...
            ~isscalar(config.minimum_segment_dt_s) || ...
            ~isfinite(config.minimum_segment_dt_s) || ...
            config.minimum_segment_dt_s < 0
        error('MonoTeach:InvalidTimedJointTrajectoryConfig', ...
            'Timing config needs finite positive context-DOF velocity limits and a nonnegative minimum dt.');
    end
end


function validate_ik_result_set(ikResultSet, dof)

    if ~isstruct(ikResultSet) || ~isscalar(ikResultSet) || ...
            ~isfield(ikResultSet, 'coordinate_space') || ...
            ~isfield(ikResultSet, 'units') || ...
            ~isfield(ikResultSet, 'segments') || ...
            ~strcmp(string(ikResultSet.coordinate_space), "joint") || ...
            ~strcmp(string(ikResultSet.units), "rad") || ...
            ~isstruct(ikResultSet.segments)
        error('MonoTeach:InvalidTimedJointIKResultSet', ...
            'Timed trajectory input must be a joint/rad IK result set.');
    end
    for segmentIndex = 1:numel(ikResultSet.segments)
        segment = ikResultSet.segments(segmentIndex);
        if ~isfield(segment, 'preik_segment_index') || ...
                ~isfield(segment, 'results') || ~isstruct(segment.results) || ...
                isempty(segment.results)
            error('MonoTeach:InvalidTimedJointIKSuccessSegment', ...
                'Every timed input segment must be a nonempty IK-success segment.');
        end
        for pointIndex = 1:numel(segment.results)
            point = segment.results(pointIndex);
            if ~isfield(point, 'source_index') || ~isfield(point, 't_ms')
                error('MonoTeach:InvalidTimedJointProvenance', ...
                    'Every timed input point needs source_index and t_ms.');
            end
            require_q(point, segmentIndex, pointIndex, dof);
        end
    end
end


function epsilon = velocity_epsilon()

    epsilon = 1.0e-12;
end


function waypoint = empty_waypoint(dof)

    waypoint = struct('q_rad', NaN(1, dof), 't_s', NaN, 'dt_s', NaN, ...
        'delta_q_rad', NaN(1, dof), 'step_velocity_rad_s', NaN(1, dof), ...
        'within_velocity_limits', false, 'segment_index', NaN, ...
        'ik_success_segment_index', NaN, 'preik_segment_index', NaN, ...
        'source_index', NaN, 'source_t_ms', NaN, 'provenance', struct());
end


function segment = empty_segment(dof)

    segment = struct('segment_index', NaN, 'ik_success_segment_index', NaN, ...
        'preik_segment_index', NaN, 'source_indices', zeros(1, 0), ...
        'waypoints', struct([]), 'q_rad', zeros(0, dof), 't_s', zeros(0, 1), ...
        'dt_s', zeros(0, 1), 'step_velocity_rad_s', zeros(0, dof), ...
        'duration_s', NaN, ...
        'max_abs_step_velocity_rad_s', NaN(1, dof));
end
