function demo_task_space_retarget(jsonPath)
%DEMO_TASK_SPACE_RETARGET Visualize one WorkspaceTrajectory in two frames.
%
% Input:
%   jsonPath - Stage 2.3 workspace trajectory JSON path
%
% The demo draws only. It does not perform IK, move the robot, or create
% executable gap segments.

    if nargin ~= 1
        error( ...
            'MonoTeach:InvalidArgumentCount', ...
            'demo_task_space_retarget requires exactly one JSON path.');
    end

    stage3Dir = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(stage3Dir);
    stage1Dir = fullfile(repoRoot, 'stage1');

    addpath(genpath(stage3Dir));
    addpath(stage1Dir);

    % Stage 2.3 workspace coordinates remain in millimetres until conversion.
    workspaceTrajectory = load_workspace_trajectory(jsonPath);
    config = default_task_plane_config();

    % Valid samples become robot-base XYZ metres; invalid gaps remain empty.
    taskTrajectory = workspace_to_task_trajectory( ...
        workspaceTrajectory, ...
        config);

    draw_workspace_figure(workspaceTrajectory);
    draw_robot_base_figure(taskTrajectory, config);
end


function draw_workspace_figure(workspaceTrajectory)

    metadata = workspaceTrajectory.metadata;
    samples = workspaceTrajectory.samples;

    figure('Name', 'MonoTeach Workspace Trajectory');
    ax = axes;
    hold(ax, 'on');

    % This rectangle and all sample coordinates are in workspace [mm].
    boundaryX = [0, metadata.width_mm, metadata.width_mm, 0, 0];
    boundaryY = [0, 0, metadata.height_mm, metadata.height_mm, 0];
    hBoundary = plot( ...
        ax, ...
        boundaryX, ...
        boundaryY, ...
        'k-', ...
        'LineWidth', 1.5, ...
        'DisplayName', '190 x 290 mm workspace');

    % Only adjacent valid samples are joined: invalid gaps must not create
    % an artificial straight-line stroke in the workspace plot.
    draw_workspace_valid_links(ax, samples);

    [insideX, insideY, outsideX, outsideY] = workspace_plot_points(samples);

    hInside = plot( ...
        ax, ...
        insideX, ...
        insideY, ...
        'bo', ...
        'MarkerFaceColor', 'b', ...
        'DisplayName', 'Valid inside workspace');

    % Outside points stay visible and unmodified; only their marker changes.
    hOutside = plot( ...
        ax, ...
        outsideX, ...
        outsideY, ...
        'rd', ...
        'MarkerFaceColor', 'r', ...
        'DisplayName', 'Valid outside workspace');

    axis(ax, 'equal');
    xlim(ax, [0, metadata.width_mm]);
    ylim(ax, [0, metadata.height_mm]);
    set(ax, 'YDir', 'reverse');
    grid(ax, 'on');
    xlabel(ax, 'Workspace X (mm)');
    ylabel(ax, 'Workspace Y (mm)');
    title(ax, 'Stage 2.3 WorkspaceTrajectory');
    legend(ax, [hBoundary, hInside, hOutside], 'Location', 'best');
end


function draw_robot_base_figure(taskTrajectory, config)

    % Robot geometry, task plane, and retargeted points are all base-frame [m].
    robot = build_legacy_robot();
    qHome = homeConfiguration(robot);

    figure('Name', 'MonoTeach Robot Base Task Trajectory');
    ax = axes;
    plot_legacy_robot(robot, qHome, ax);
    hold(ax, 'on');

    taskPlaneCornersBaseM = task_plane_corners_base_m(config);
    hTaskPlane = plot3( ...
        ax, ...
        taskPlaneCornersBaseM(:, 1), ...
        taskPlaneCornersBaseM(:, 2), ...
        taskPlaneCornersBaseM(:, 3), ...
        'm-', ...
        'LineWidth', 2, ...
        'DisplayName', 'Scaled task plane');

    hAnchor = plot3( ...
        ax, ...
        config.robot_anchor_m(1), ...
        config.robot_anchor_m(2), ...
        config.robot_anchor_m(3), ...
        'kp', ...
        'MarkerSize', 12, ...
        'LineWidth', 2, ...
        'DisplayName', 'Task-plane anchor');

    samples = taskTrajectory.samples;

    % The same valid-neighbour-only rule preserves visual gaps after retargeting.
    draw_task_valid_links(ax, samples);

    [insidePointsM, outsidePointsM] = task_plot_points(samples);

    hInside = plot3( ...
        ax, ...
        insidePointsM(:, 1), ...
        insidePointsM(:, 2), ...
        insidePointsM(:, 3), ...
        'bo', ...
        'MarkerFaceColor', 'b', ...
        'DisplayName', 'Valid retargeted point');

    hOutside = plot3( ...
        ax, ...
        outsidePointsM(:, 1), ...
        outsidePointsM(:, 2), ...
        outsidePointsM(:, 3), ...
        'rd', ...
        'MarkerFaceColor', 'r', ...
        'DisplayName', 'Valid but outside workspace');

    % Workspace and base plots have corresponding shapes. Their scale differs
    % because this view applies mm-to-m scaling and task +Y maps to base -Z.
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'Base X (m)');
    ylabel(ax, 'Base Y (m)');
    zlabel(ax, 'Base Z (m)');
    title(ax, 'Stage 3.1A TaskTrajectory in Legacy Robot Base Frame');
    view(ax, 3);
    legend( ...
        ax, ...
        [hTaskPlane, hAnchor, hInside, hOutside], ...
        'Location', 'best');
end


function draw_workspace_valid_links(ax, samples)

    for i = 2:numel(samples)
        previous = samples(i - 1);
        current = samples(i);

        if previous.valid && current.valid
            plot( ...
                ax, ...
                [previous.x_mm, current.x_mm], ...
                [previous.y_mm, current.y_mm], ...
                '-', ...
                'Color', [0.20, 0.45, 0.80], ...
                'LineWidth', 1.5, ...
                'HandleVisibility', 'off');
        end
    end
end


function draw_task_valid_links(ax, samples)

    for i = 2:numel(samples)
        previous = samples(i - 1);
        current = samples(i);

        if previous.valid && current.valid
            plot3( ...
                ax, ...
                [previous.x_m, current.x_m], ...
                [previous.y_m, current.y_m], ...
                [previous.z_m, current.z_m], ...
                '-', ...
                'Color', [0.20, 0.45, 0.80], ...
                'LineWidth', 1.5, ...
                'HandleVisibility', 'off');
        end
    end
end


function [insideX, insideY, outsideX, outsideY] = workspace_plot_points(samples)

    insideX = [];
    insideY = [];
    outsideX = [];
    outsideY = [];

    for i = 1:numel(samples)
        sample = samples(i);

        if ~sample.valid
            continue;
        end

        if sample.inside_workspace
            insideX(end + 1) = sample.x_mm; %#ok<AGROW>
            insideY(end + 1) = sample.y_mm; %#ok<AGROW>
        else
            outsideX(end + 1) = sample.x_mm; %#ok<AGROW>
            outsideY(end + 1) = sample.y_mm; %#ok<AGROW>
        end
    end
end


function [insidePointsM, outsidePointsM] = task_plot_points(samples)

    insidePointsM = zeros(0, 3);
    outsidePointsM = zeros(0, 3);

    for i = 1:numel(samples)
        sample = samples(i);

        if ~sample.valid
            continue;
        end

        pointM = [sample.x_m, sample.y_m, sample.z_m];

        if sample.inside_workspace
            insidePointsM(end + 1, :) = pointM; %#ok<AGROW>
        else
            outsidePointsM(end + 1, :) = pointM; %#ok<AGROW>
        end
    end
end


function cornersBaseM = task_plane_corners_base_m(config)

    halfWidthM = config.workspace_width_mm * config.scale / 2;
    halfHeightM = config.workspace_height_mm * config.scale / 2;

    cornersTaskPlaneM = [ ...
        -halfWidthM, -halfHeightM, 0; ...
         halfWidthM, -halfHeightM, 0; ...
         halfWidthM,  halfHeightM, 0; ...
        -halfWidthM,  halfHeightM, 0; ...
        -halfWidthM, -halfHeightM, 0];

    RBaseTaskplane = config.T_base_taskplane(1:3, 1:3);
    anchorBaseM = config.T_base_taskplane(1:3, 4)';

    cornersBaseM = ...
        (RBaseTaskplane * cornersTaskPlaneM' + anchorBaseM')';
end
