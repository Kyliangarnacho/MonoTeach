function sweep = demo_writing_task_plane_scale_sweep(sweep)
%DEMO_WRITING_TASK_PLANE_SCALE_SWEEP Plot coarse scale coverage evidence.

    if nargin == 0
        stage3Dir = fileparts(fileparts(mfilename('fullpath')));
        repoRoot = fileparts(stage3Dir);
        addpath(genpath(stage3Dir));
        addpath(fullfile(repoRoot, 'stage1'));
        sweep = sweep_writing_task_plane_scale( ...
            build_legacy_robot(), ...
            default_task_plane_config(), ...
            default_ik_config(), ...
            default_writing_posture_config(), ...
            default_writing_scale_sweep_config());
    end

    records = sweep.coarse_records;
    scaleMM = 1000.0 * [records.scale_m_per_mm];
    widthsMM = 1000.0 * [records.robot_physical_width_m];
    heightsMM = 1000.0 * [records.robot_physical_height_m];

    figure('Name', 'MonoTeach Stage 3.3 Uniform Writing Scale Sweep');
    tiledlayout(1, 3);
    nexttile;
    plot(scaleMM, [records.coverage_percent], '-o');
    xlabel('Uniform scale (mm / workspace mm)');
    ylabel('Coarse sampled coverage (%)');
    ylim([0, 100]);
    grid on;

    nexttile;
    plot(scaleMM, widthsMM, '-o', scaleMM, heightsMM, '-s');
    xlabel('Uniform scale (mm / workspace mm)');
    ylabel('Robot Task Plane size (mm)');
    legend('width', 'height', 'Location', 'northwest');
    grid on;

    nexttile;
    plot(scaleMM, [records.worst_accepted_direction_error_deg], '-o');
    xlabel('Uniform scale (mm / workspace mm)');
    ylabel('Worst accepted direction error (deg)');
    grid on;
end
