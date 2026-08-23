function demo = demo_constrained_path_gap_recovery(run)
%DEMO_CONSTRAINED_PATH_GAP_RECOVERY Plot strict and recovered path evidence.
%
% One task-space figure only.  Separate plot calls preserve every remaining
% failure gap; recovered XYZ points are plotted at their unchanged targets.

    if nargin < 1 || isempty(run)
        run = run_real_constrained_path_gap_recovery();
    end
    validate_input(run);
    figureHandle = figure('Name', 'MonoTeach Constrained Path Gap Recovery V1');
    axesHandle = axes(figureHandle);
    hold(axesHandle, 'on'); grid(axesHandle, 'on'); axis(axesHandle, 'equal');
    draw_reference_path(axesHandle, run.baseline.resampled_preik_adapter);
    draw_strict_segments(axesHandle, run.strict_writing_result_set);
    draw_recovered_points(axesHandle, run.recovery.window_results);
    xlabel(axesHandle, 'Robot base X (m)');
    ylabel(axesHandle, 'Robot base Y (m)');
    zlabel(axesHandle, 'Robot base Z (m)');
    title(axesHandle, 'Writing reference, strict segments, and orientation-relaxed points');
    legend(axesHandle, 'Location', 'best');
    view(axesHandle, 3);
    demo = struct('run', run, 'figure', figureHandle);
end


function draw_reference_path(axesHandle, adapter)

    for index = 1:numel(adapter.segments)
        samples = adapter.segments(index).samples;
        xyz = [[samples.x_m]' [samples.y_m]' [samples.z_m]'];
        name = 'Writing reference path';
        if index > 1, name = ''; end
        plot3(axesHandle, xyz(:, 1), xyz(:, 2), xyz(:, 3), '-', ...
            'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.1, ...
            'DisplayName', name);
    end
end


function draw_strict_segments(axesHandle, strict)

    for index = 1:numel(strict.segments)
        points = strict.segments(index).results;
        xyz = vertcat(points.target_xyz_m);
        name = 'Strict Writing accepted segment';
        if index > 1, name = ''; end
        plot3(axesHandle, xyz(:, 1), xyz(:, 2), xyz(:, 3), 'o-', ...
            'Color', [0.00, 0.45, 0.74], 'MarkerFaceColor', [0.00, 0.45, 0.74], ...
            'DisplayName', name);
    end
end


function draw_recovered_points(axesHandle, results)

    first = true;
    for windowIndex = 1:numel(results)
        points = results(windowIndex).recovered_points;
        if isempty(points), continue; end
        xyz = vertcat(points.original_target_xyz_m);
        name = '';
        if first, name = 'Orientation-relaxed recovered point'; first = false; end
        plot3(axesHandle, xyz(:, 1), xyz(:, 2), xyz(:, 3), 's', ...
            'Color', [0.85, 0.33, 0.10], 'MarkerFaceColor', [0.85, 0.33, 0.10], ...
            'MarkerSize', 7, 'DisplayName', name);
    end
end


function validate_input(run)

    if ~isstruct(run) || ~isfield(run, 'baseline') || ...
            ~isfield(run, 'strict_writing_result_set') || ...
            ~isfield(run, 'recovery')
        error('MonoTeach:InvalidConstrainedGapRecoveryDemoInput', ...
            'Demo requires a real constrained gap-recovery run artifact.');
    end
end
