function resampled = resample_task_trajectory_arclength(taskTrajectory, config)
%RESAMPLE_TASK_TRAJECTORY_ARCLENGTH Derived uniform arc-length waypoint runs.
%
% Only contiguous valid && inside_workspace runs are resampled.  The source
% TaskTrajectory remains unchanged; output points are explicitly interpolated
% derived samples with bracket and interpolation provenance.

    if nargin < 2 || isempty(config)
        config = default_task_resampling_config();
    end
    validate_inputs(taskTrajectory, config);
    sourceBefore = taskTrajectory;
    segmentSet = build_preik_segments(taskTrajectory);
    runs = repmat(empty_run(), 1, 0);
    nextResampledIndex = 1;
    for runIndex = 1:numel(segmentSet.segments)
        sourceRun = segmentSet.segments(runIndex);
        [points, nextResampledIndex, statistics] = resample_one_run( ...
            sourceRun, runIndex, config.target_spacing_m, nextResampledIndex);
        runs(end + 1) = struct( ...
            'original_run_index', runIndex, ...
            'source_indices', sourceRun.source_indices, ...
            'source_point_count', numel(sourceRun.samples), ...
            'resampled_point_count', numel(points), ...
            'arc_length_m', statistics.arc_length_m, ...
            'spacing_statistics_m', statistics.spacing_statistics_m, ...
            'points', points); %#ok<AGROW>
    end
    if ~isequal(taskTrajectory, sourceBefore)
        error('MonoTeach:TaskTrajectoryMutationDetected', ...
            'Arc-length resampling must not modify its TaskTrajectory input.');
    end

    resampled = struct();
    resampled.artifact_type = 'task_trajectory_arclength_resampling';
    resampled.coordinate_frame = taskTrajectory.coordinate_frame;
    resampled.units = 'm';
    resampled.source_task_trajectory_metadata = taskTrajectory.metadata;
    resampled.target_spacing_m = config.target_spacing_m;
    resampled.runs = runs;
    resampled.summary = summarize_runs(runs, numel(taskTrajectory.samples));
end


function [points, nextIndex, statistics] = resample_one_run( ...
        sourceRun, runIndex, spacingM, nextIndex)

    samples = sourceRun.samples;
    sourceIndices = sourceRun.source_indices;
    xyz = [[samples.x_m]' [samples.y_m]' [samples.z_m]'];
    tMs = [samples.t_ms];
    if numel(samples) == 1
        points = make_point(xyz(1, :), tMs(1), nextIndex, ...
            sourceIndices(1), sourceIndices(1), 0.0, runIndex, 0.0);
        nextIndex = nextIndex + 1;
        statistics = struct('arc_length_m', 0.0, ...
            'spacing_statistics_m', spacing_statistics(zeros(1, 0)));
        return;
    end

    segmentLengths = vecnorm(diff(xyz, 1, 1), 2, 2)';
    cumulative = [0.0, cumsum(segmentLengths)];
    totalLength = cumulative(end);
    queryArc = uniform_query_arclengths(totalLength, spacingM);
    points = repmat(empty_point(), 1, 0);
    for queryIndex = 1:numel(queryArc)
        s = queryArc(queryIndex);
        [left, ratio] = locate_bracket(cumulative, s);
        right = left + 1;
        pointXYZ = (1.0 - ratio) * xyz(left, :) + ratio * xyz(right, :);
        pointTMs = (1.0 - ratio) * tMs(left) + ratio * tMs(right);
        points(end + 1) = make_point(pointXYZ, pointTMs, nextIndex, ...
            sourceIndices(left), sourceIndices(right), ratio, runIndex, s); %#ok<AGROW>
        nextIndex = nextIndex + 1;
    end
    outputXYZ = vertcat(points.xyz_m);
    statistics = struct('arc_length_m', totalLength, ...
        'spacing_statistics_m', spacing_statistics( ...
            vecnorm(diff(outputXYZ, 1, 1), 2, 2)'));
end


function queryArc = uniform_query_arclengths(totalLength, spacingM)

    if totalLength <= eps
        % Coincident endpoints are represented once rather than creating a
        % synthetic zero-length waypoint interval.
        queryArc = 0.0;
        return;
    end
    queryArc = 0.0:spacingM:totalLength;
    if totalLength - queryArc(end) > 1e-12
        queryArc(end + 1) = totalLength;
    else
        queryArc(end) = totalLength;
    end
end


function [left, ratio] = locate_bracket(cumulative, s)

    segmentCount = numel(cumulative) - 1;
    if s >= cumulative(end)
        left = segmentCount;
        ratio = 1.0;
        return;
    end
    left = find(cumulative(1:end-1) <= s & s < cumulative(2:end), 1, 'last');
    if isempty(left)
        left = 1;
    end
    lengthM = cumulative(left + 1) - cumulative(left);
    if lengthM <= eps
        ratio = 0.0;
    else
        ratio = (s - cumulative(left)) / lengthM;
    end
end


function point = make_point( ...
        xyzM, tMs, resampledIndex, leftSourceIndex, rightSourceIndex, ...
        ratio, runIndex, arclengthM)

    point = struct( ...
        'resampled_index', resampledIndex, ...
        'xyz_m', xyzM, ...
        'interpolated_t_ms', tMs, ...
        'source_bracket_indices', [leftSourceIndex, rightSourceIndex], ...
        'interpolation_ratio', ratio, ...
        'original_run_index', runIndex, ...
        'source_arclength_m', arclengthM);
end


function point = empty_point()

    point = struct('resampled_index', NaN, 'xyz_m', NaN(1, 3), ...
        'interpolated_t_ms', NaN, 'source_bracket_indices', NaN(1, 2), ...
        'interpolation_ratio', NaN, 'original_run_index', NaN, ...
        'source_arclength_m', NaN);
end


function run = empty_run()

    run = struct('original_run_index', NaN, 'source_indices', zeros(1, 0), ...
        'source_point_count', 0, 'resampled_point_count', 0, ...
        'arc_length_m', NaN, 'spacing_statistics_m', spacing_statistics([]), ...
        'points', struct([]));
end


function summary = summarize_runs(runs, sourceSampleCount)

    allDistances = zeros(1, 0);
    for runIndex = 1:numel(runs)
        allDistances = [allDistances, ...
            runs(runIndex).spacing_statistics_m.distances_m]; %#ok<AGROW>
    end
    summary = struct();
    summary.source_task_sample_count = sourceSampleCount;
    summary.eligible_run_count = numel(runs);
    summary.source_eligible_point_count = sum([runs.source_point_count]);
    summary.resampled_point_count = sum([runs.resampled_point_count]);
    summary.spacing_statistics_m = spacing_statistics(allDistances);
    summary.per_run_point_counts = [runs.resampled_point_count];
end


function statistics = spacing_statistics(distancesM)

    statistics = struct('distances_m', distancesM, 'count', numel(distancesM), ...
        'min_m', NaN, 'mean_m', NaN, 'max_m', NaN);
    if ~isempty(distancesM)
        statistics.min_m = min(distancesM);
        statistics.mean_m = mean(distancesM);
        statistics.max_m = max(distancesM);
    end
end


function validate_inputs(taskTrajectory, config)

    if ~isstruct(taskTrajectory) || ~isscalar(taskTrajectory) || ...
            ~isfield(taskTrajectory, 'coordinate_frame') || ...
            ~strcmp(string(taskTrajectory.coordinate_frame), "robot_base") || ...
            ~isfield(taskTrajectory, 'metadata') || ...
            ~isfield(taskTrajectory.metadata, 'units') || ...
            ~strcmp(string(taskTrajectory.metadata.units), "m") || ...
            ~isfield(taskTrajectory, 'samples') || ~isstruct(config) || ...
            ~isfield(config, 'target_spacing_m') || ...
            ~isscalar(config.target_spacing_m) || ...
            ~isfinite(config.target_spacing_m) || config.target_spacing_m <= 0
        error('MonoTeach:InvalidTaskArcLengthResamplingInput', ...
            'Expected robot-base metre TaskTrajectory and positive spacing.');
    end
end
