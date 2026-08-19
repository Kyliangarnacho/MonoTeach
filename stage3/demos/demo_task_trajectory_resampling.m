function demo = demo_task_trajectory_resampling(taskTrajectory, config)
%DEMO_TASK_TRAJECTORY_RESAMPLING Plot source and derived arc-length runs.

    if nargin < 2 || isempty(config)
        config = default_task_resampling_config();
    end
    resampled = resample_task_trajectory_arclength(taskTrajectory, config);
    figureHandle = figure('Name', 'MonoTeach Task-Trajectory Arc-Length Resampling');
    ax = axes(figureHandle);
    hold(ax, 'on');
    colors = lines(max(numel(resampled.runs), 1));
    for runIndex = 1:numel(resampled.runs)
        run = resampled.runs(runIndex);
        sourceSamples = taskTrajectory.samples(run.source_indices);
        sourceXYZ = [[sourceSamples.x_m]' [sourceSamples.y_m]' [sourceSamples.z_m]'];
        resampledXYZ = vertcat(run.points.xyz_m);
        plot3(ax, sourceXYZ(:,1), sourceXYZ(:,2), sourceXYZ(:,3), 'o-', ...
            'Color', colors(runIndex,:), 'DisplayName', ...
            sprintf('Original run %d (%d)', runIndex, run.source_point_count));
        plot3(ax, resampledXYZ(:,1), resampledXYZ(:,2), resampledXYZ(:,3), 's--', ...
            'Color', colors(runIndex,:), 'MarkerFaceColor', 'w', ...
            'DisplayName', sprintf('Resampled run %d (%d)', ...
            runIndex, run.resampled_point_count));
    end
    axis(ax, 'equal'); grid(ax, 'on'); view(ax, [0, 0]);
    xlabel(ax, 'Robot base X (m)'); ylabel(ax, 'Robot base Y (m)');
    zlabel(ax, 'Robot base Z (m)');
    statistics = resampled.summary.spacing_statistics_m;
    title(ax, sprintf(['Piecewise-linear arc-length resampling, h=%.4f m; ' ...
        'distance min/mean/max=%.4g/%.4g/%.4g m'], ...
        config.target_spacing_m, statistics.min_m, statistics.mean_m, ...
        statistics.max_m));
    if ~isempty(resampled.runs), legend(ax, 'Location', 'best'); end
    demo = struct('resampled_trajectory', resampled, 'figure', figureHandle);
end
