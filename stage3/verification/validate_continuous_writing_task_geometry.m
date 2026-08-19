function validation = validate_continuous_writing_task_geometry(robot, ...
        continuousTrajectory, writingResultSet, taskPlaneConfig, ...
        candidateConfig, postureConfig, robotContext)
%VALIDATE_CONTINUOUS_WRITING_TASK_GEOMETRY FK-check ContinuousJointTrajectory.
%
% The reference path is deliberately only a piecewise-linear interpolation of
% the original accepted Writing target XYZ values at the continuous segment's
% final waypoint times.  This is a validation artifact, not a new planner.
% Every continuous IK-success segment remains independent.

    if nargin < 7 || isempty(robotContext)
        robotContext = legacy_compatibility_context(robot);
    end
    robotModel = robotContext.model;
    validate_inputs(robotModel, continuousTrajectory, writingResultSet, ...
        taskPlaneConfig, candidateConfig, postureConfig, robotContext);
    signedNormal = candidateConfig.normal_sign * ...
        derive_task_plane_normal(taskPlaneConfig)';
    limits = robotContext.joint_limits;
    segments = repmat(empty_segment(robotContext.dof), 1, 0);
    for index = 1:numel(continuousTrajectory.segments)
        continuousSegment = continuousTrajectory.segments(index);
        writingSegment = writingResultSet.segments( ...
            continuousSegment.ik_success_segment_index);
        segments(end + 1) = validate_one_segment(robotModel, continuousSegment, ...
            writingSegment, signedNormal, limits, robotContext.end_effector, ...
            robotContext.dof); %#ok<AGROW>
    end
    validation = struct();
    validation.artifact_type = 'ContinuousJointTrajectoryTaskGeometryValidation';
    validation.coordinate_frame = 'robot_base';
    validation.units = 'm_rad';
    validation.robot_id = robotContext.id;
    validation.end_effector = robotContext.end_effector;
    validation.tool_axis = sprintf('%s_local_%s', ...
        robotContext.end_effector, postureConfig.tool_axis);
    validation.signed_normal_base = signedNormal;
    validation.reference_path_method = ...
        'piecewise_linear_original_writing_waypoints_in_final_waypoint_time';
    validation.segments = segments;
    validation.summary = summarize(segments);
end


function segment = validate_one_segment(robot, continuousSegment, ...
        writingSegment, signedNormal, limits, endEffector, dof)

    writingResults = writingSegment.results;
    referenceWaypointXYZ = vertcat(writingResults.target_xyz_m);
    waypointTimes = continuousSegment.waypoint_t_s(:);
    sampleTimes = continuousSegment.t_s(:);
    referenceXYZ = interpolate_reference(waypointTimes, referenceWaypointXYZ, sampleTimes);
    sampleCount = numel(sampleTimes);
    fkXYZ = zeros(sampleCount, 3);
    toolZ = zeros(sampleCount, 3);
    for sampleIndex = 1:sampleCount
        pose = getTransform(robot, continuousSegment.q_rad(sampleIndex, :), endEffector);
        fkXYZ(sampleIndex, :) = pose(1:3, 4)';
        toolZ(sampleIndex, :) = unit_row(pose(1:3, 3)');
    end
    positionDeviation = vecnorm(fkXYZ - referenceXYZ, 2, 2);
    directionError = direction_errors(toolZ, signedNormal);
    margin = min(continuousSegment.q_rad - limits(:,1)', ...
        limits(:,2)' - continuousSegment.q_rad);
    withinLimits = all(continuousSegment.q_rad >= limits(:,1)' - limit_epsilon() & ...
        continuousSegment.q_rad <= limits(:,2)' + limit_epsilon(), 2);
    [maxDeviation, worstIndex] = max(positionDeviation);
    [maxDirectionError, worstDirectionIndex] = max(directionError);

    segment = empty_segment(dof);
    segment.segment_index = continuousSegment.segment_index;
    segment.ik_success_segment_index = continuousSegment.ik_success_segment_index;
    segment.preik_segment_index = continuousSegment.preik_segment_index;
    segment.source_indices = continuousSegment.source_indices;
    segment.waypoint_t_s = waypointTimes;
    segment.waypoint_target_xyz_m = referenceWaypointXYZ;
    segment.waypoint_provenance = continuousSegment.waypoint_provenance;
    segment.t_s = sampleTimes;
    segment.q_rad = continuousSegment.q_rad;
    segment.reference_xyz_m = referenceXYZ;
    segment.fk_xyz_m = fkXYZ;
    segment.fk_tool_z_direction = toolZ;
    segment.position_deviation_m = positionDeviation;
    segment.tool_direction_error_rad = directionError;
    segment.tool_direction_error_deg = rad2deg(directionError);
    segment.within_joint_limits = withinLimits;
    segment.joint_limit_margin = margin;
    segment.mean_position_deviation_m = mean(positionDeviation);
    segment.max_position_deviation_m = maxDeviation;
    segment.max_tool_direction_error_rad = maxDirectionError;
    segment.max_tool_direction_error_deg = max(segment.tool_direction_error_deg);
    segment.minimum_joint_limit_margin = min(margin, [], 'all');
    segment.has_joint_limit_violation = any(~withinLimits);
    segment.worst_position_sample_index = worstIndex;
    segment.worst_position_time_s = sampleTimes(worstIndex);
    segment.worst_tool_direction_sample_index = worstDirectionIndex;
    segment.worst_tool_direction_time_s = sampleTimes(worstDirectionIndex);
end


function referenceXYZ = interpolate_reference(waypointTimes, waypointXYZ, sampleTimes)

    referenceXYZ = interp1(waypointTimes, waypointXYZ, sampleTimes, 'linear');
    if any(~isfinite(referenceXYZ), 'all')
        error('MonoTeach:ContinuousReferenceInterpolationFailure', ...
            'Reference path interpolation must remain finite within waypoint time bounds.');
    end
end


function errors = direction_errors(toolZ, signedNormal)

    dotProducts = toolZ * signedNormal';
    errors = acos(max(-1.0, min(1.0, dotProducts)));
end


function summary = summarize(segments)

    summary = struct();
    summary.validated_segment_count = numel(segments);
    summary.validated_sample_count = sum(arrayfun( ...
        @(segment) numel(segment.t_s), segments));
    if isempty(segments)
        summary.mean_cartesian_path_deviation_m = NaN;
        summary.max_cartesian_path_deviation_m = NaN;
        summary.max_tool_direction_error_rad = NaN;
        summary.max_tool_direction_error_deg = NaN;
        summary.minimum_joint_limit_margin = NaN;
        summary.has_continuous_joint_limit_violation = false;
        summary.worst_sample_segment_index = [];
        summary.worst_sample_index = [];
        summary.worst_sample_time_s = [];
        summary.worst_tool_direction_segment_index = [];
        summary.worst_tool_direction_sample_index = [];
        summary.worst_tool_direction_time_s = [];
        return;
    end
    deviations = vertcat(segments.position_deviation_m);
    summary.mean_cartesian_path_deviation_m = mean(deviations);
    [summary.max_cartesian_path_deviation_m, flatIndex] = max(deviations);
    sampleCounts = arrayfun(@(segment) numel(segment.t_s), segments);
    cumulative = cumsum(sampleCounts);
    segmentIndex = find(flatIndex <= cumulative, 1, 'first');
    prior = 0;
    if segmentIndex > 1, prior = cumulative(segmentIndex - 1); end
    localIndex = flatIndex - prior;
    summary.max_tool_direction_error_rad = max( ...
        vertcat(segments.tool_direction_error_rad));
    summary.max_tool_direction_error_deg = rad2deg( ...
        summary.max_tool_direction_error_rad);
    summary.minimum_joint_limit_margin = min(vertcat( ...
        segments.joint_limit_margin), [], 'all');
    summary.has_continuous_joint_limit_violation = any([ ...
        segments.has_joint_limit_violation]);
    summary.worst_sample_segment_index = segments(segmentIndex).segment_index;
    summary.worst_sample_index = localIndex;
    summary.worst_sample_time_s = segments(segmentIndex).t_s(localIndex);
    directions = vertcat(segments.tool_direction_error_rad);
    [~, flatDirectionIndex] = max(directions);
    directionSegmentIndex = find(flatDirectionIndex <= cumulative, 1, 'first');
    directionPrior = 0;
    if directionSegmentIndex > 1, directionPrior = cumulative(directionSegmentIndex - 1); end
    directionLocalIndex = flatDirectionIndex - directionPrior;
    summary.worst_tool_direction_segment_index = ...
        segments(directionSegmentIndex).segment_index;
    summary.worst_tool_direction_sample_index = directionLocalIndex;
    summary.worst_tool_direction_time_s = ...
        segments(directionSegmentIndex).t_s(directionLocalIndex);
end


function validate_inputs(robot, continuous, writing, taskPlane, candidate, posture, context)

    if ~isa(robot, 'rigidBodyTree') || ~strcmp(robot.DataFormat, 'row') || ...
            ~isstruct(context) || ~isscalar(context) || ...
            ~all(isfield(context, {'id', 'model', 'dof', 'end_effector', ...
            'joint_limits', 'backend'})) || context.dof < 1 || ...
            ~isequal(size(context.joint_limits), [context.dof, 2]) || ...
            ~isstruct(continuous) || ...
            ~isfield(continuous, 'artifact_type') || ...
            ~strcmp(string(continuous.artifact_type), "ContinuousJointTrajectory") || ...
            ~isfield(continuous, 'segments') || ~isstruct(continuous.segments) || ...
            ~isstruct(writing) || ~isfield(writing, 'segments') || ...
            ~isstruct(writing.segments) || ~isstruct(taskPlane) || ...
            ~isstruct(candidate) || candidate.normal_sign ~= +1 || ...
            ~isstruct(posture) || ...
            ~strcmp(string(posture.end_effector), string(context.end_effector)) || ...
            ~strcmp(string(posture.tool_axis), "z")
        error('MonoTeach:InvalidContinuousWritingGeometryInput', ...
            'Validation requires matching RobotContext, Writing result set, and +normal candidate.');
    end
    for index = 1:numel(continuous.segments)
        segment = continuous.segments(index);
        required = {'segment_index', 'ik_success_segment_index', ...
            'preik_segment_index', 'source_indices', 'waypoint_t_s', ...
            'waypoint_provenance', 't_s', 'q_rad'};
        if ~all(isfield(segment, required)) || ...
                segment.ik_success_segment_index < 1 || ...
                segment.ik_success_segment_index > numel(writing.segments) || ...
                ~isequal(size(segment.q_rad, 2), [context.dof]) || ...
                numel(segment.t_s) ~= size(segment.q_rad, 1) || ...
                numel(segment.waypoint_t_s) ~= numel(segment.source_indices) || ...
                any(diff(segment.t_s(:)) <= 0) || ...
                any(diff(segment.waypoint_t_s(:)) <= 0) || ...
                any(~isfinite(segment.q_rad), 'all')
            error('MonoTeach:InvalidContinuousWritingGeometrySegment', ...
                'Continuous segment %d has invalid timing or context-DOF samples.', index);
        end
        results = writing.segments(segment.ik_success_segment_index).results;
        if numel(results) ~= numel(segment.source_indices) || ...
                ~isequal([results.source_index], segment.source_indices) || ...
                ~all(isfield(results, 'target_xyz_m'))
            error('MonoTeach:ContinuousWritingWaypointMismatch', ...
                'Continuous segment %d must match its original Writing targets.', index);
        end
    end
end


function value = unit_row(vector)

    magnitude = norm(vector);
    if ~isfinite(magnitude) || magnitude <= eps
        error('MonoTeach:DegenerateContinuousFKToolDirection', ...
            'Tool local-Z direction from FK must be finite and nonzero.');
    end
    value = vector / magnitude;
end


function epsilon = limit_epsilon()

    epsilon = 1.0e-12;
end


function context = legacy_compatibility_context(robot)

    context = load_robot_context("legacy5");
    if ~isa(robot, 'rigidBodyTree') || ...
            numel(homeConfiguration(robot)) ~= context.dof
        error('MonoTeach:InvalidLegacyValidationRobot', ...
            'Compatibility validation expects the Legacy5 rigidBodyTree.');
    end
    context.model = robot;
end


function segment = empty_segment(dof)

    segment = struct('segment_index', NaN, 'ik_success_segment_index', NaN, ...
        'preik_segment_index', NaN, 'source_indices', zeros(1, 0), ...
        'waypoint_t_s', zeros(0, 1), 'waypoint_target_xyz_m', zeros(0, 3), ...
        'waypoint_provenance', {cell(0, 1)}, 't_s', zeros(0, 1), ...
        'q_rad', zeros(0, dof), 'reference_xyz_m', zeros(0, 3), ...
        'fk_xyz_m', zeros(0, 3), 'fk_tool_z_direction', zeros(0, 3), ...
        'position_deviation_m', zeros(0, 1), ...
        'tool_direction_error_rad', zeros(0, 1), ...
        'tool_direction_error_deg', zeros(0, 1), ...
        'within_joint_limits', false(0, 1), 'joint_limit_margin', zeros(0, dof), ...
        'mean_position_deviation_m', NaN, 'max_position_deviation_m', NaN, ...
        'max_tool_direction_error_rad', NaN, ...
        'max_tool_direction_error_deg', NaN, ...
        'minimum_joint_limit_margin', NaN, 'has_joint_limit_violation', false, ...
        'worst_position_sample_index', NaN, 'worst_position_time_s', NaN, ...
        'worst_tool_direction_sample_index', NaN, ...
        'worst_tool_direction_time_s', NaN);
end
