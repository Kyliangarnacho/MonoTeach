function scan = demo_writing_full_task_plane_scan(scan)
%DEMO_WRITING_FULL_TASK_PLANE_SCAN Plot coarse full-plane sampled coverage.

    if nargin == 0
        stage3Dir = fileparts(fileparts(mfilename('fullpath')));
        repoRoot = fileparts(stage3Dir);
        addpath(genpath(stage3Dir));
        addpath(fullfile(repoRoot, 'stage1'));
        scan = scan_writing_full_task_plane( ...
            build_legacy_robot(), ...
            default_task_plane_config(), ...
            default_ik_config(), ...
            default_writing_posture_config(), ...
            default_writing_full_plane_scan_config());
    end

    records = scan.records;
    xValues = unique([records.workspace_x_mm]);
    yValues = unique([records.workspace_y_mm]);
    figure('Name', 'MonoTeach Stage 3.3 Full Task Plane Feasibility');
    tiledlayout(1, 3);
    draw_map(nexttile, xValues, yValues, metric_map( ...
        records, yValues, xValues, 'accepted'), 'Sampled PASS / FAIL', 'Accepted');
    draw_map(nexttile, xValues, yValues, metric_map( ...
        records, yValues, xValues, 'direction_error_deg'), ...
        'Direction error', 'deg');
    draw_map(nexttile, xValues, yValues, metric_map( ...
        records, yValues, xValues, 'minimum_joint_limit_margin'), ...
        'Minimum joint-limit margin', 'rad');
end


function values = metric_map(records, yValues, xValues, fieldName)

    values = NaN(numel(yValues), numel(xValues));
    for recordIndex = 1:numel(records)
        row = find(yValues == records(recordIndex).workspace_y_mm, 1);
        column = find(xValues == records(recordIndex).workspace_x_mm, 1);
        values(row, column) = double(records(recordIndex).(fieldName));
    end
end


function draw_map(ax, xValues, yValues, values, titleText, colorLabel)

    imagesc(ax, xValues, yValues, values);
    set(ax, 'YDir', 'normal');
    xlabel(ax, 'Workspace X (mm)');
    ylabel(ax, 'Workspace Y (mm)');
    title(ax, titleText);
    colorbarHandle = colorbar(ax);
    colorbarHandle.Label.String = colorLabel;
end
