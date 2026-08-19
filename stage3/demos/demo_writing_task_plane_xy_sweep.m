function sweep = demo_writing_task_plane_xy_sweep(sweep)
%DEMO_WRITING_TASK_PLANE_XY_SWEEP Plot Step 7 XY anchor feasibility maps.

    if nargin == 0
        stage3Dir = fileparts(fileparts(mfilename('fullpath')));
        repoRoot = fileparts(stage3Dir);
        addpath(genpath(stage3Dir));
        addpath(fullfile(repoRoot, 'stage1'));
        sweep = sweep_writing_task_plane_xy( ...
            build_legacy_robot(), ...
            default_task_plane_config(), ...
            default_ik_config(), ...
            default_writing_posture_config(), ...
            default_writing_xy_sweep_config());
    end

    records = sweep.records;
    xValues = unique([records.x_m]);
    yValues = unique([records.y_m]);
    figure('Name', 'MonoTeach Stage 3.3 Local XY Feasibility');
    tiledlayout(1, 3);
    draw_map(nexttile, xValues, yValues, metric_map( ...
        records, yValues, xValues, 'accepted'), 'Accepted (+normal)', 'Accepted');
    draw_map(nexttile, xValues, yValues, log10(metric_map( ...
        records, yValues, xValues, 'position_error_m')), ...
        'Position error (+normal)', 'log10(m)');
    draw_map(nexttile, xValues, yValues, metric_map( ...
        records, yValues, xValues, 'direction_error_deg'), ...
        'Direction error (+normal)', 'deg');
end


function values = metric_map(records, yValues, xValues, fieldName)

    values = NaN(numel(yValues), numel(xValues));
    for recordIndex = 1:numel(records)
        row = find(yValues == records(recordIndex).y_m, 1);
        column = find(xValues == records(recordIndex).x_m, 1);
        values(row, column) = double(records(recordIndex).(fieldName));
    end
end


function draw_map(ax, xValues, yValues, values, titleText, colorLabel)

    imagesc(ax, xValues, yValues, values);
    set(ax, 'YDir', 'normal');
    xlabel(ax, 'Task Plane center X (m)');
    ylabel(ax, 'Task Plane center Y (m)');
    title(ax, titleText);
    colorbarHandle = colorbar(ax);
    colorbarHandle.Label.String = colorLabel;
end
