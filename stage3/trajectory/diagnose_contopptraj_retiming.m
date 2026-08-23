function diagnostic = diagnose_contopptraj_retiming( ...
        timedTrajectory, continuousTrajectory, robotContext, sampleCount)
%DIAGNOSE_CONTOPPTRAJ_RETIMING Compare TOPP-RA with the frozen Quintic path.
%
% This is a diagnostic adapter only.  It never changes TimedJointTrajectory
% or the formal quintic artifact.  contopptraj receives each existing
% Quintic piecewise path, so only the time allocation is reconsidered.

    if nargin < 3 || isempty(robotContext)
        robotContext = load_robot_context("legacy5");
    end
    if nargin < 4 || isempty(sampleCount)
        sampleCount = 500;
    end
    validate_inputs(timedTrajectory, continuousTrajectory, robotContext, sampleCount);

    diagnostic = struct();
    diagnostic.artifact_type = 'ContopptrajRetimingDiagnostic';
    diagnostic.method = 'matlab_contopptraj_toppra';
    diagnostic.path_source = 'existing_quintic_piecewise_polynomial';
    diagnostic.robot_id = robotContext.id;
    diagnostic.available = exist('contopptraj', 'file') == 2;
    diagnostic.segments = repmat(empty_segment(robotContext.dof), 1, 0);

    if ~diagnostic.available
        diagnostic.unavailable_reason = 'contopptraj_not_available_on_matlab_path';
        diagnostic.summary = empty_summary();
        return;
    end
    diagnostic.unavailable_reason = '';

    for index = 1:numel(continuousTrajectory.segments)
        baseline = continuousTrajectory.segments(index);
        if ~isfield(baseline, 'piecewise_polynomial') || ...
                ~isstruct(baseline.piecewise_polynomial) || ...
                ~isfield(baseline.piecewise_polynomial, 'breaks')
            error('MonoTeach:MissingContopptrajPath', ...
                'Continuous segment %d does not expose a piecewise path.', index);
        end
        diagnostic.segments(end + 1) = retime_one_segment( ...
            timedTrajectory.segments(index), baseline, robotContext, sampleCount); %#ok<AGROW>
    end
    diagnostic.summary = summarize(diagnostic.segments);
end


function segment = retime_one_segment(timedSegment, baseline, context, sampleCount)

    velocityLimits = [-context.velocity_limits(:), context.velocity_limits(:)];
    accelerationLimits = [-context.acceleration_limits(:), ...
        context.acceleration_limits(:)];
    [q, qd, qdd, t, solverInfo] = contopptraj( ...
        baseline.piecewise_polynomial, velocityLimits, accelerationLimits, ...
        NumSamples=sampleCount);
    t = t(:);
    q = q';
    qd = qd';
    qdd = qdd';

    pathParameter = invert_piecewise_path_parameter( ...
        baseline.piecewise_polynomial, q);
    baselineQ = ppval(baseline.piecewise_polynomial, pathParameter)';
    commonPathJointDeviation = vecnorm(q - baselineQ, 2, 2);
    retimedFK = evaluate_fk_xyz(context, q);
    commonPathFK = evaluate_fk_xyz(context, baselineQ);
    commonPathFKDeviation = vecnorm(retimedFK - commonPathFK, 2, 2);

    temporalDeviation = NaN(size(q, 1), 1);
    temporalJointDeviation = NaN(size(q, 1), 1);
    overlap = t >= baseline.t_s(1) - 1e-12 & ...
        t <= baseline.t_s(end) + 1e-12;
    if any(overlap)
        temporalBaselineQ = ppval( ...
            baseline.piecewise_polynomial, t(overlap))';
        temporalJointDeviation(overlap) = vecnorm( ...
            q(overlap, :) - temporalBaselineQ, 2, 2);
        temporalBaselineFK = evaluate_fk_xyz(context, temporalBaselineQ);
        temporalDeviation(overlap) = vecnorm( ...
            retimedFK(overlap, :) - temporalBaselineFK, 2, 2);
    end

    denseParameter = linspace( ...
        baseline.piecewise_polynomial.breaks(1), ...
        baseline.piecewise_polynomial.breaks(end), ...
        max(2000, 4 * sampleCount));
    denseQ = ppval(baseline.piecewise_polynomial, denseParameter)';
    denseFK = evaluate_fk_xyz(context, denseQ);
    retimedToBaselineDistance = point_to_polyline_distance(retimedFK, denseFK);
    baselineToRetimedDistance = point_to_polyline_distance(denseFK, retimedFK);

    margin = min(q - context.joint_limits(:, 1)', ...
        context.joint_limits(:, 2)' - q);
    segment = empty_segment(context.dof);
    segment.segment_index = baseline.segment_index;
    segment.ik_success_segment_index = baseline.ik_success_segment_index;
    segment.preik_segment_index = baseline.preik_segment_index;
    segment.source_indices = timedSegment.source_indices;
    segment.waypoint_provenance = baseline.waypoint_provenance;
    segment.t_s = t;
    segment.q_rad = q;
    segment.qd_rad_s = qd;
    segment.qdd_rad_s2 = qdd;
    segment.solver_info = solverInfo;
    segment.duration_s = t(end) - t(1);
    segment.peak_velocity_rad_s = max(abs(qd), [], 1);
    segment.peak_acceleration_rad_s2 = max(abs(qdd), [], 1);
    segment.velocity_limits_satisfied = all(segment.peak_velocity_rad_s <= ...
        context.velocity_limits + 1e-10);
    segment.acceleration_limits_satisfied = all(segment.peak_acceleration_rad_s2 <= ...
        context.acceleration_limits + 1e-10);
    segment.joint_limits_satisfied = all(q >= context.joint_limits(:, 1)' - 1e-10 & ...
        q <= context.joint_limits(:, 2)' + 1e-10, 'all');
    segment.joint_limit_margin = margin;
    segment.minimum_joint_limit_margin = min(margin, [], 'all');
    segment.path_parameter = pathParameter;
    segment.common_path_joint_deviation_rad = commonPathJointDeviation;
    segment.common_path_fk_deviation_m = commonPathFKDeviation;
    segment.temporal_reference_joint_deviation_rad = temporalJointDeviation;
    segment.temporal_reference_deviation_m = temporalDeviation;
    segment.geometric_fk_distance_to_baseline_m = retimedToBaselineDistance;
    segment.geometric_fk_distance_from_baseline_m = baselineToRetimedDistance;
    segment.path_joint_deviation_rad = commonPathJointDeviation;
    segment.fk_path_deviation_m = commonPathFKDeviation;
    segment.mean_fk_path_deviation_m = mean(commonPathFKDeviation);
    segment.max_fk_path_deviation_m = max(commonPathFKDeviation);
    segment.mean_temporal_reference_deviation_m = mean(temporalDeviation, 'omitnan');
    segment.max_temporal_reference_deviation_m = max(temporalDeviation, [], 'omitnan');
    segment.max_geometric_fk_distance_m = max([ ...
        retimedToBaselineDistance; baselineToRetimedDistance]);
end


function summary = summarize(segments)

    summary = empty_summary();
    summary.segment_count = numel(segments);
    if isempty(segments)
        return;
    end
    summary.total_duration_s = sum([segments.duration_s]);
    summary.peak_velocity_rad_s = max(vertcat(segments.peak_velocity_rad_s), [], 1);
    summary.peak_acceleration_rad_s2 = max(vertcat(segments.peak_acceleration_rad_s2), [], 1);
    summary.minimum_joint_limit_margin = min([segments.minimum_joint_limit_margin]);
    summary.mean_fk_path_deviation_m = mean([segments.mean_fk_path_deviation_m]);
    summary.max_fk_path_deviation_m = max([segments.max_fk_path_deviation_m]);
    summary.mean_temporal_reference_deviation_m = mean([ ...
        segments.mean_temporal_reference_deviation_m]);
    summary.max_temporal_reference_deviation_m = max([ ...
        segments.max_temporal_reference_deviation_m]);
    summary.max_geometric_fk_distance_m = max([segments.max_geometric_fk_distance_m]);
    summary.max_joint_space_geometric_deviation_rad = max(vertcat( ...
        segments.common_path_joint_deviation_rad), [], 'all');
    summary.velocity_limits_satisfied = all([segments.velocity_limits_satisfied]);
    summary.acceleration_limits_satisfied = all([segments.acceleration_limits_satisfied]);
    summary.joint_limits_satisfied = all([segments.joint_limits_satisfied]);
end


function summary = empty_summary()

    summary = struct('segment_count', 0, 'total_duration_s', NaN, ...
        'peak_velocity_rad_s', [], 'peak_acceleration_rad_s2', [], ...
        'minimum_joint_limit_margin', NaN, 'mean_fk_path_deviation_m', NaN, ...
        'max_fk_path_deviation_m', NaN, ...
        'mean_temporal_reference_deviation_m', NaN, ...
        'max_temporal_reference_deviation_m', NaN, ...
        'max_geometric_fk_distance_m', NaN, ...
        'max_joint_space_geometric_deviation_rad', NaN, ...
        'velocity_limits_satisfied', false, ...
        'acceleration_limits_satisfied', false, 'joint_limits_satisfied', false);
end


function segment = empty_segment(dof)

    segment = struct('segment_index', NaN, 'ik_success_segment_index', NaN, ...
        'preik_segment_index', NaN, 'source_indices', [], ...
        'waypoint_provenance', {{}}, 't_s', zeros(0, 1), ...
        'q_rad', zeros(0, dof), 'qd_rad_s', zeros(0, dof), ...
        'qdd_rad_s2', zeros(0, dof), 'solver_info', struct(), ...
        'duration_s', NaN, 'peak_velocity_rad_s', NaN(1, dof), ...
        'peak_acceleration_rad_s2', NaN(1, dof), ...
        'velocity_limits_satisfied', false, ...
        'acceleration_limits_satisfied', false, ...
        'joint_limits_satisfied', false, 'joint_limit_margin', NaN(0, dof), ...
        'minimum_joint_limit_margin', NaN, ...
        'path_joint_deviation_rad', zeros(0, 1), ...
        'fk_path_deviation_m', zeros(0, 1), ...
        'path_parameter', zeros(0, 1), ...
        'common_path_joint_deviation_rad', zeros(0, 1), ...
        'common_path_fk_deviation_m', zeros(0, 1), ...
        'temporal_reference_joint_deviation_rad', zeros(0, 1), ...
        'temporal_reference_deviation_m', zeros(0, 1), ...
        'geometric_fk_distance_to_baseline_m', zeros(0, 1), ...
        'geometric_fk_distance_from_baseline_m', zeros(0, 1), ...
        'mean_fk_path_deviation_m', NaN, 'max_fk_path_deviation_m', NaN, ...
        'mean_temporal_reference_deviation_m', NaN, ...
        'max_temporal_reference_deviation_m', NaN, ...
        'max_geometric_fk_distance_m', NaN);
end


function validate_inputs(timed, continuous, context, sampleCount)

    if ~isstruct(timed) || ~isfield(timed, 'segments') || ...
            ~isstruct(continuous) || ~isfield(continuous, 'segments') || ...
            numel(timed.segments) ~= numel(continuous.segments) || ...
            ~isstruct(context) || ~all(isfield(context, ...
            {'id', 'model', 'dof', 'end_effector', 'joint_limits', ...
            'velocity_limits', 'acceleration_limits'})) || ...
            ~isscalar(sampleCount) || sampleCount < 2 || sampleCount ~= floor(sampleCount)
        error('MonoTeach:InvalidContopptrajInput', ...
            'TOPP-RA diagnostic requires matching trajectory segments and RobotContext limits.');
    end
end


function pathParameter = invert_piecewise_path_parameter(pp, q)

    breaks = pp.breaks(:)';
    pathParameter = zeros(size(q, 1), 1);
    options = optimset('Display', 'off');
    for sampleIndex = 1:size(q, 1)
        bestParameter = breaks(1);
        bestError = inf;
        for intervalIndex = 1:numel(breaks) - 1
            lower = breaks(intervalIndex);
            upper = breaks(intervalIndex + 1);
            objective = @(parameter) sum(( ...
                ppval(pp, parameter) - q(sampleIndex, :)').^2);
            [candidate, candidateError] = fminbnd( ...
                objective, lower, upper, options);
            endpointParameters = [lower, upper, candidate];
            endpointErrors = arrayfun(objective, endpointParameters);
            [intervalError, localIndex] = min(endpointErrors);
            if intervalError < bestError
                bestError = intervalError;
                bestParameter = endpointParameters(localIndex);
            end
            if candidateError < bestError
                bestError = candidateError;
                bestParameter = candidate;
            end
        end
        pathParameter(sampleIndex) = bestParameter;
    end
end


function xyz = evaluate_fk_xyz(context, q)

    xyz = zeros(size(q, 1), 3);
    for sampleIndex = 1:size(q, 1)
        pose = getTransform(context.model, q(sampleIndex, :), context.end_effector);
        xyz(sampleIndex, :) = pose(1:3, 4)';
    end
end


function distances = point_to_polyline_distance(points, polyline)

    distances = zeros(size(points, 1), 1);
    if size(polyline, 1) < 2
        distances(:) = NaN;
        return;
    end
    startPoints = polyline(1:end-1, :);
    edgeVectors = diff(polyline, 1, 1);
    edgeSquared = sum(edgeVectors.^2, 2);
    edgeSquared(edgeSquared < eps) = 1;
    for pointIndex = 1:size(points, 1)
        offsets = points(pointIndex, :) - startPoints;
        ratios = sum(offsets .* edgeVectors, 2) ./ edgeSquared;
        ratios = min(max(ratios, 0), 1);
        projections = startPoints + ratios .* edgeVectors;
        distances(pointIndex) = min(vecnorm( ...
            projections - points(pointIndex, :), 2, 2));
    end
end