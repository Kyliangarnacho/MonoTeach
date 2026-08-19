function sweep = demo_writing_anchor_height_sweep()
%DEMO_WRITING_ANCHOR_HEIGHT_SWEEP Plot coarse writing-anchor height evidence.
%
% The three panels show position error, direction error, and formal
% accepted/rejected state for both task-plane normal candidates.

    stage3Dir = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(stage3Dir);
    addpath(genpath(stage3Dir));
    addpath(fullfile(repoRoot, 'stage1'));

    robot = build_legacy_robot();
    sweep = sweep_writing_anchor_height( ...
        robot, ...
        default_task_plane_config(), ...
        default_ik_config(), ...
        default_writing_posture_config(), ...
        default_writing_height_sweep_config());

    plot_sweep(sweep);
end


function plot_sweep(sweep)

    figure('Name', 'MonoTeach Stage 3.3 Writing Anchor Height Sweep');
    signs = [+1, -1];
    colors = [0.10, 0.40, 0.90; 0.85, 0.15, 0.15];
    labels = {'+normal', '-normal'};

    tiledlayout(3, 1);
    for panelIndex = 1:3
        ax = nexttile;
        hold(ax, 'on');
        for signIndex = 1:numel(signs)
            records = sweep.records([sweep.records.sign] == signs(signIndex));
            z = [records.anchor_z_m];
            switch panelIndex
                case 1
                    values = [records.position_error_m];
                    semilogy(ax, z, values, '-o', ...
                        'Color', colors(signIndex, :), ...
                        'DisplayName', labels{signIndex});
                    ylabel(ax, 'Position error (m)');
                case 2
                    values = [records.direction_error_deg];
                    plot(ax, z, values, '-o', ...
                        'Color', colors(signIndex, :), ...
                        'DisplayName', labels{signIndex});
                    ylabel(ax, 'Direction error (deg)');
                case 3
                    values = double([records.accepted]);
                    stairs(ax, z, values, '-o', ...
                        'Color', colors(signIndex, :), ...
                        'DisplayName', labels{signIndex});
                    ylabel(ax, 'Accepted (1=yes)');
                    ylim(ax, [-0.1, 1.1]);
            end
        end
        grid(ax, 'on');
        legend(ax, 'Location', 'best');
        if panelIndex == 3
            xlabel(ax, 'Anchor Base Z (m)');
        end
    end
end
