function sweep = demo_writing_task_plane_yaw_sweep(sweep)
%DEMO_WRITING_TASK_PLANE_YAW_SWEEP Plot writing feasibility vs yaw and height.

    if nargin == 0
        stage3Dir = fileparts(fileparts(mfilename('fullpath')));
        repoRoot = fileparts(stage3Dir);
        addpath(genpath(stage3Dir));
        addpath(fullfile(repoRoot, 'stage1'));

        sweep = sweep_writing_task_plane_yaw( ...
            build_legacy_robot(), ...
            default_task_plane_config(), ...
            default_ik_config(), ...
            default_writing_posture_config(), ...
            default_writing_yaw_sweep_config());
    end

    figure('Name', 'MonoTeach Stage 3.3 Vertical Yaw-Height Feasibility');
    tiledlayout(2, 3);
    for signIndex = 1:2
        signValue = [+1, -1];
        signValue = signValue(signIndex);
        records = sweep.records([sweep.records.normal_sign] == signValue);
        yaws = unique([records.yaw_deg]);
        heights = unique([records.center_z_m]);
        draw_map(nexttile, yaws, heights, metric_map(records, heights, yaws, 'accepted'), ...
            sprintf('%+dnormal accepted (1=yes)', signValue), 'Accepted');
        draw_map(nexttile, yaws, heights, log10(metric_map( ...
            records, heights, yaws, 'position_error_m')), ...
            sprintf('%+dnormal log10 position error', signValue), 'log10(m)');
        draw_map(nexttile, yaws, heights, metric_map( ...
            records, heights, yaws, 'direction_error_deg'), ...
            sprintf('%+dnormal direction error', signValue), 'deg');
    end
end


function values = metric_map(records, heights, yaws, fieldName)

    values = NaN(numel(heights), numel(yaws));
    for recordIndex = 1:numel(records)
        row = find(heights == records(recordIndex).center_z_m, 1);
        column = find(yaws == records(recordIndex).yaw_deg, 1);
        values(row, column) = double(records(recordIndex).(fieldName));
    end
end


function draw_map(ax, yaws, heights, values, titleText, colorLabel)

    imagesc(ax, yaws, heights, values);
    set(ax, 'YDir', 'normal');
    xlabel(ax, 'Yaw about Base Z (deg)');
    ylabel(ax, 'Center Z (m)');
    title(ax, titleText);
    colorbarHandle = colorbar(ax);
    colorbarHandle.Label.String = colorLabel;
end
