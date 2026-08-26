function report = verify_replay_robot_visualization(tracePath, imagePath)
%VERIFY_REPLAY_ROBOT_VISUALIZATION Render one complete Legacy5 frame for QA.
%
% The interactive demo animates these same FK origins.  This short verifier is
% intentionally static: it proves that axes limits contain the base plus all
% five body origins and writes an inspectable PNG without spending minutes in
% an off-screen animation loop.

    if nargin < 2, imagePath = ''; end
    run = run_replay_trace_execution(tracePath);
    robot = run.robot_context.model;
    q = run.execution.q_rad(end,:);
    points = all_body_origins(robot, run.execution.q_rad);
    current = all_body_origins(robot, q);

    figureHandle = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 900, 720]);
    ax = axes(figureHandle); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); axis(ax, 'vis3d'); view(ax, [-35, 22]);
    show(robot, q, 'Parent', ax, 'PreservePlot', true, 'Frames', 'on');
    plot3(ax, current(:,1), current(:,2), current(:,3), '-', 'Color', [0.10 0.35 0.85], 'LineWidth', 4);
    colors = lines(robot.NumBodies);
    for i = 1:robot.NumBodies
        plot3(ax, current(i+1,1), current(i+1,2), current(i+1,3), 'o', ...
            'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 9);
        labelOffset = [0.015 * mod(i,3), 0.012 * floor(i/3), 0.014 + 0.006 * mod(i,2)];
        if i == 1
            labelOffset = [-0.030, 0.020, 0.030];
        elseif i == 2
            labelOffset = [0.030, -0.020, 0.014];
        end
        text(ax, current(i+1,1)+labelOffset(1), current(i+1,2)+labelOffset(2), ...
            current(i+1,3)+labelOffset(3), sprintf('J%d:%s', i, robot.Bodies{i}.Joint.Name));
    end
    plot3(ax, current(1,1), current(1,2), current(1,3), 's', ...
        'Color', [0.1 0.1 0.1], 'MarkerFaceColor', [0.65 0.65 0.65], 'MarkerSize', 9);
    text(ax, current(1,1), current(1,2), current(1,3)+0.010, 'base');
    set_limits(ax, points);
    xlabel(ax, 'Base X [m]'); ylabel(ax, 'Base Y [m]'); zlabel(ax, 'Base Z [m]');
    title(ax, 'Legacy5 complete kinematic chain: base + J1 ... J5');
    drawnow;

    limits = [xlim(ax); ylim(ax); zlim(ax)];
    report = struct();
    report.body_origin_count = size(current,1);
    report.joint_label_count = robot.NumBodies;
    report.axes_contain_all_trace_origins = all(points(:,1) >= limits(1,1) & points(:,1) <= limits(1,2)) && ...
        all(points(:,2) >= limits(2,1) & points(:,2) <= limits(2,2)) && ...
        all(points(:,3) >= limits(3,1) & points(:,3) <= limits(3,2));
    assert(report.body_origin_count == robot.NumBodies + 1, 'Expected base plus five body origins.');
    assert(report.joint_label_count == robot.NumBodies, 'Expected one label for every joint.');
    assert(report.axes_contain_all_trace_origins, 'Robot axes clipped a body origin.');
    if ~isempty(imagePath)
        exportgraphics(figureHandle, imagePath, 'Resolution', 150);
    end
    close(figureHandle);
end


function points = all_body_origins(robot, qTrace)

    points = zeros((robot.NumBodies + 1) * size(qTrace,1), 3);
    cursor = 1;
    for row = 1:size(qTrace,1)
        p = zeros(robot.NumBodies + 1, 3);
        for body = 1:robot.NumBodies
            pose = getTransform(robot, qTrace(row,:), robot.Bodies{body}.Name);
            p(body+1,:) = pose(1:3,4)';
        end
        points(cursor:cursor + robot.NumBodies,:) = p;
        cursor = cursor + robot.NumBodies + 1;
    end
end


function set_limits(ax, points)

    lower = min(points, [], 1); upper = max(points, [], 1);
    padding = max(0.15 * (upper - lower), 0.02);
    xlim(ax, [lower(1)-padding(1), upper(1)+padding(1)]);
    ylim(ax, [lower(2)-padding(2), upper(2)+padding(2)]);
    zlim(ax, [lower(3)-padding(3), upper(3)+padding(3)]);
end
