function visualizer = live_legacy5_visualizer(action, varargin)
%LIVE_LEGACY5_VISUALIZER Persistent full-chain renderer for the live executor.
%
% ``rigidBodyTree/show`` has no useful mesh for this educational Legacy5
% model, so a blank axes does not imply failed IK.  This helper deliberately
% draws the same FK body-origin skeleton, labels, and local frames that made
% the accepted Stage 3.4 replay animation inspectable.  It creates graphics
% once and later only changes their data; control scheduling never rebuilds a
% figure at 100 Hz.

    action = string(action);
    switch action
        case "create"
            robot = varargin{1}; q = varargin{2}; ax = varargin{3};
            visualizer = create_visualizer(robot, q, ax);
        case "update"
            visualizer = varargin{1}; robot = varargin{2}; q = varargin{3};
            visualizer = update_visualizer(visualizer, robot, q);
        case "trails"
            visualizer = varargin{1}; reference = varargin{2}; executed = varargin{3};
            visualizer = update_trails(visualizer, reference, executed);
        otherwise
            error('MonoTeach:LiveVisualizerAction', 'Unknown visualizer action %s.', action);
    end
end


function visualizer = create_visualizer(robot, q, ax)

    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal'); axis(ax, 'vis3d');
    view(ax, [-35, 22]); xlabel(ax, 'Base X [m]'); ylabel(ax, 'Base Y [m]'); zlabel(ax, 'Base Z [m]');
    p = body_points_live(robot, q);
    colors = lines(robot.NumBodies);
    visualizer.link = plot3(ax, p(:,1), p(:,2), p(:,3), '-', 'Color', [0.10 0.35 0.85], 'LineWidth', 4, 'DisplayName', 'Legacy5 skeleton');
    % The black trail is Cartesian q(t)'s desired end-effector path after
    % workspace->robot-base retargeting.  The green trail is FK(q) sampled
    % from what the simulated executor actually consumed.  Keeping both
    % makes a tracking error visible instead of hiding it in a joint plot.
    visualizer.referenceTrail = plot3(ax, NaN, NaN, NaN, 'k--', 'LineWidth', 1.2, 'DisplayName', 'Cartesian q(t) reference');
    visualizer.executedTrail = plot3(ax, NaN, NaN, NaN, '-', 'Color', [0.0 0.75 0.15], 'LineWidth', 2.0, 'DisplayName', 'Executed FK trail');
    visualizer.endEffector = plot3(ax, p(end,1), p(end,2), p(end,3), 'o', 'Color', [0.0 0.55 0.10], 'MarkerFaceColor', [0.0 0.80 0.15], 'MarkerSize', 7, 'DisplayName', 'Current end effector');
    visualizer.markers = gobjects(robot.NumBodies + 1, 1);
    visualizer.markers(1) = plot3(ax, p(1,1), p(1,2), p(1,3), 's', 'Color', [0.1 0.1 0.1], 'MarkerFaceColor', [0.65 0.65 0.65], 'MarkerSize', 8, 'DisplayName', 'base');
    for i = 1:robot.NumBodies
        visualizer.markers(i+1) = plot3(ax, p(i+1,1), p(i+1,2), p(i+1,3), 'o', 'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 9, 'DisplayName', sprintf('J%d', i));
    end
    offsets = label_offsets_live(robot.NumBodies + 1);
    visualizer.labels = gobjects(robot.NumBodies + 1, 1);
    visualizer.labels(1) = text(ax, p(1,1)+offsets(1,1), p(1,2)+offsets(1,2), p(1,3)+offsets(1,3), 'base', 'FontWeight', 'bold');
    for i = 1:robot.NumBodies
        visualizer.labels(i+1) = text(ax, p(i+1,1)+offsets(i+1,1), p(i+1,2)+offsets(i+1,2), p(i+1,3)+offsets(i+1,3), sprintf('J%d:%s', i, robot.Bodies{i}.Joint.Name), 'FontSize', 8);
    end
    [origins, rotations] = body_frames_live(robot, q);
    colors = [0.85 0.20 0.20; 0.15 0.60 0.20; 0.15 0.35 0.90];
    visualizer.frames = gobjects(robot.NumBodies, 3);
    for i = 1:robot.NumBodies
        for axisIndex = 1:3
            d = rotations(:,axisIndex,i)' * 0.018;
            visualizer.frames(i,axisIndex) = quiver3(ax, origins(i,1), origins(i,2), origins(i,3), d(1), d(2), d(3), 0, 'Color', colors(axisIndex,:), 'LineWidth', 1.0, 'MaxHeadSize', 0.4, 'HandleVisibility', 'off');
        end
    end
    visualizer.lower = min(p, [], 1); visualizer.upper = max(p, [], 1);
    visualizer = update_visualizer(visualizer, robot, q);
end


function visualizer = update_visualizer(visualizer, robot, q)

    p = body_points_live(robot, q);
    set(visualizer.link, 'XData', p(:,1), 'YData', p(:,2), 'ZData', p(:,3));
    for i = 1:numel(visualizer.markers)
        set(visualizer.markers(i), 'XData', p(i,1), 'YData', p(i,2), 'ZData', p(i,3));
    end
    offsets = label_offsets_live(numel(visualizer.labels));
    for i = 1:numel(visualizer.labels)
        set(visualizer.labels(i), 'Position', p(i,:) + offsets(i,:));
    end
    [origins, rotations] = body_frames_live(robot, q);
    for i = 1:robot.NumBodies
        for axisIndex = 1:3
            d = rotations(:,axisIndex,i)' * 0.018;
            set(visualizer.frames(i,axisIndex), 'XData', origins(i,1), 'YData', origins(i,2), 'ZData', origins(i,3), 'UData', d(1), 'VData', d(2), 'WData', d(3));
        end
    end
    visualizer.lower = min([visualizer.lower; p], [], 1);
    visualizer.upper = max([visualizer.upper; p], [], 1);
    padding = max(0.15 * (visualizer.upper - visualizer.lower), 0.02);
    ax = ancestor(visualizer.link, 'axes');
    xlim(ax, [visualizer.lower(1)-padding(1), visualizer.upper(1)+padding(1)]);
    ylim(ax, [visualizer.lower(2)-padding(2), visualizer.upper(2)+padding(2)]);
    zlim(ax, [visualizer.lower(3)-padding(3), visualizer.upper(3)+padding(3)]);
end


function visualizer = update_trails(visualizer, reference, executed)

    if isempty(reference), reference = zeros(0,3); end
    if isempty(executed), executed = zeros(0,3); end
    set(visualizer.referenceTrail, 'XData', reference(:,1), 'YData', reference(:,2), 'ZData', reference(:,3));
    set(visualizer.executedTrail, 'XData', executed(:,1), 'YData', executed(:,2), 'ZData', executed(:,3));
    if ~isempty(executed)
        set(visualizer.endEffector, 'XData', executed(end,1), 'YData', executed(end,2), 'ZData', executed(end,3));
    end
end


function p = body_points_live(robot, q)
    p = zeros(robot.NumBodies + 1, 3);
    for i = 1:robot.NumBodies
        pose = getTransform(robot, q, robot.Bodies{i}.Name);
        p(i+1,:) = pose(1:3,4)';
    end
end


function [origins, rotations] = body_frames_live(robot, q)
    origins = zeros(robot.NumBodies, 3); rotations = zeros(3, 3, robot.NumBodies);
    for i = 1:robot.NumBodies
        pose = getTransform(robot, q, robot.Bodies{i}.Name);
        origins(i,:) = pose(1:3,4)'; rotations(:,:,i) = pose(1:3,1:3);
    end
end


function offsets = label_offsets_live(count)
    offsets = zeros(count, 3);
    for i = 1:count
        offsets(i,:) = [0.015 * mod(i-1, 3), 0.012 * floor((i-1)/3), 0.014 + 0.006 * mod(i-1, 2)];
    end
    if count >= 3
        offsets(2,:) = [-0.030, 0.020, 0.030];
        offsets(3,:) = [0.030, -0.020, 0.014];
    end
end
