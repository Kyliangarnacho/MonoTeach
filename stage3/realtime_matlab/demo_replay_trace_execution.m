function demo = demo_replay_trace_execution(tracePath, playbackRate, config)
%DEMO_REPLAY_TRACE_EXECUTION Inspect virtual source, q(t) FIFO, and every joint.
%
% run_replay_trace_execution finishes the deterministic 100 Hz simulation
% before this function draws anything.  playbackRate and render_rate_hz change
% only human viewing speed: q(t), source timestamps, queue order, and reported
% execution times are already fixed in ``run``.

    if nargin < 2 || isempty(playbackRate), playbackRate = 1.0; end
    if nargin < 3, config = []; end
    if ~isscalar(playbackRate) || ~isfinite(playbackRate) || playbackRate <= 0
        error('MonoTeach:InvalidReplayPlaybackRate', 'playbackRate must be positive finite.');
    end
    run = run_replay_trace_execution(tracePath, config);
    trace = run.trace;
    execution = run.execution;
    robot = run.robot_context.model;
    renderStride = max(1, round(1.0 / ...
        (run.config.render_rate_hz * run.config.controller_period_s)));
    renderIndices = unique([1:renderStride:numel(execution.t_s), numel(execution.t_s)]);

    figureHandle = figure('Name', 'MonoTeach Stage 3.4 Replay -> q(t) -> Legacy5', ...
        'Color', 'w', 'Position', [80, 60, 1500, 980]);
    layout = tiledlayout(3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    workspaceAxes = nexttile(layout, 1); hold(workspaceAxes, 'on'); grid(workspaceAxes, 'on'); axis(workspaceAxes, 'equal');
    robotAxes = nexttile(layout, 2); hold(robotAxes, 'on'); grid(robotAxes, 'on'); axis(robotAxes, 'equal'); axis(robotAxes, 'vis3d'); view(robotAxes, [-35, 22]);
    timelineAxes = nexttile(layout, 3, [1, 2]); hold(timelineAxes, 'on'); grid(timelineAxes, 'on');
    jointAxes = nexttile(layout, 5, [1, 2]); hold(jointAxes, 'on'); grid(jointAxes, 'on');

    [sourceXY, sourceT] = source_workspace_points(trace.source_events);
    plot(workspaceAxes, sourceXY(:,1), sourceXY(:,2), ':', 'Color', [0.65 0.65 0.65], ...
        'DisplayName', 'Recorded source points');
    publishedHandle = plot(workspaceAxes, NaN, NaN, 'bo-', 'LineWidth', 1.2, ...
        'MarkerSize', 4, 'DisplayName', 'Published by ReplaySource');
    plannedXY = cartesian_trace_points(trace.planned_segments);
    plot(workspaceAxes, plannedXY(:,1), plannedXY(:,2), 'r-', 'LineWidth', 1.4, ...
        'DisplayName', 'Causal Cartesian quintics');
    plot_prepare_observations(workspaceAxes, trace);
    xlabel(workspaceAxes, 'Workspace X [mm]'); ylabel(workspaceAxes, 'Workspace Y [mm]');
    title(workspaceAxes, 'Virtual camera publication and Cartesian planning'); legend(workspaceAxes, 'Location', 'best');

    validDesired = all(isfinite(execution.desired_base_xyz_m), 2);
    plot3(robotAxes, execution.desired_base_xyz_m(validDesired,1), ...
        execution.desired_base_xyz_m(validDesired,2), execution.desired_base_xyz_m(validDesired,3), ...
        'k--', 'LineWidth', 1.2, 'DisplayName', 'Cartesian q(t) reference');
    fkTrail = plot3(robotAxes, NaN, NaN, NaN, 'g-', 'LineWidth', 1.6, 'DisplayName', 'Executed FK trail');
    xlabel(robotAxes, 'Base X [m]'); ylabel(robotAxes, 'Base Y [m]'); zlabel(robotAxes, 'Base Z [m]');
    title(robotAxes, 'Legacy5: all body origins and joint frames'); legend(robotAxes, 'Location', 'best');
    robotHandles = render_robot(robot, execution.q_rad(1,:), robotAxes);
    skeletonHandle = render_skeleton(robot, execution.q_rad(1,:), robotAxes);
    jointLabelHandles = render_joint_labels(robot, execution.q_rad(1,:), robotAxes);
    jointFrameHandles = render_joint_frames(robot, execution.q_rad(1,:), robotAxes);
    % End-effector-only limits clipped the base and low joints.  Every body
    % origin over the full trace is included, so all five joints stay visible.
    allBodyOrigins = robot_body_extent_points(robot, execution.q_rad);
    set_robot_limits(robotAxes, [allBodyOrigins; execution.fk_xyz_m; execution.desired_base_xyz_m(validDesired,:)]);

    sourceLines = xline(timelineAxes, sourceT / 1000.0, ':', 'Color', [0.5 0.5 0.9], ...
        'HandleVisibility', 'off'); %#ok<NASGU>
    plot_prepare_release_lines(timelineAxes, trace);
    yyaxis(timelineAxes, 'left');
    backlogLine = plot(timelineAxes, execution.t_s, execution.execution_backlog_s, 'm-', ...
        'LineWidth', 1.5, 'DisplayName', 'q(t) execution backlog [s]');
    plot(timelineAxes, execution.t_s, execution.startup_remaining_s, '--', ...
        'Color', [0.20 0.70 0.85], 'LineWidth', 1.2, ...
        'DisplayName', 'Home-to-Start remaining [s]');
    ylabel(timelineAxes, 'Backlog duration [s]');
    yyaxis(timelineAxes, 'right');
    queueCountLine = stairs(timelineAxes, execution.t_s, execution.queued_chunk_count, 'Color', [0.35 0.35 0.35], ...
        'LineWidth', 1.0, 'DisplayName', 'Waiting q(t) chunk count'); %#ok<NASGU>
    ylabel(timelineAxes, 'Waiting chunk count');
    currentTimeLine = xline(timelineAxes, 0.0, 'r-', 'LineWidth', 1.4, 'DisplayName', 'Controller time');
    xline(timelineAxes, run.summary.source_finished_replay_s, 'k:', 'LineWidth', 1.2, ...
        'DisplayName', 'Last source release');
    xlabel(timelineAxes, 'Virtual execution time [s]');
    title(timelineAxes, 'Source releases (blue ticks) and execution backlog');
    legend(timelineAxes, 'Location', 'northwest');

    jointLines = gobjects(1, run.robot_context.dof);
    for jointIndex = 1:run.robot_context.dof
        jointLines(jointIndex) = plot(jointAxes, execution.t_s, execution.q_rad(:,jointIndex), ...
            'LineWidth', 1.15, 'DisplayName', run.robot_context.joint_names{jointIndex});
    end
    jointCursor = xline(jointAxes, 0.0, 'r-', 'LineWidth', 1.4, 'DisplayName', 'Controller time');
    xlabel(jointAxes, 'Virtual execution time [s]'); ylabel(jointAxes, 'Joint position q [rad]');
    title(jointAxes, 'Every controllable Legacy5 joint: q_1(t) ... q_5(t)');
    legend(jointAxes, 'Location', 'eastoutside');

    textHandle = annotation(figureHandle, 'textbox', [0.02 0.005 0.96 0.035], ...
        'EdgeColor', 'none', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    wallClock = tic;
    for renderIndex = 1:numel(renderIndices)
        i = renderIndices(renderIndex);
        desiredWall = execution.t_s(i) / playbackRate;
        remaining = desiredWall - toc(wallClock);
        if remaining > 0, pause(remaining); end
        publishedCount = min(execution.source_published_count(i), size(sourceXY, 1));
        set(publishedHandle, 'XData', sourceXY(1:publishedCount,1), ...
            'YData', sourceXY(1:publishedCount,2));
        delete(robotHandles(isgraphics(robotHandles)));
        robotHandles = render_robot(robot, execution.q_rad(i,:), robotAxes);
        update_skeleton(skeletonHandle, robot, execution.q_rad(i,:));
        update_joint_labels(jointLabelHandles, robot, execution.q_rad(i,:));
        update_joint_frames(jointFrameHandles, robot, execution.q_rad(i,:));
        set(fkTrail, 'XData', execution.fk_xyz_m(1:i,1), 'YData', execution.fk_xyz_m(1:i,2), ...
            'ZData', execution.fk_xyz_m(1:i,3));
        currentTimeLine.Value = execution.t_s(i);
        jointCursor.Value = execution.t_s(i);
        set(textHandle, 'String', sprintf([ ...
            'execution t = %.2f s | replay t = %.0f ms | published = %d/%d | ' ...
            'ready = %d/%d | backlog = %.3f s | waiting chunks = %d | phase = %s | active = %s'], ...
            execution.t_s(i), execution.replay_t_ms(i), execution.source_published_count(i), ...
            numel(trace.source_events), execution.segments_ready_count(i), ...
            numel(trace.planned_segments), execution.execution_backlog_s(i), ...
            execution.queued_chunk_count(i), char(execution.phase(i)), ...
            active_text(execution.active_segment_index(i))));
        drawnow;
    end
    demo = struct('run', run, 'figure', figureHandle, 'workspace_axes', workspaceAxes, ...
        'robot_axes', robotAxes, 'timeline_axes', timelineAxes, 'joint_axes', jointAxes, ...
        'backlog_line', backlogLine, 'joint_lines', jointLines, 'render_stride', renderStride);
end


function plot_prepare_observations(ax, trace)

    if ~isfield(trace, 'startup') || ~isfield(trace.startup, 'prepare_observations') || ...
            isempty(trace.startup.prepare_observations)
        return;
    end
    observations = trace.startup.prepare_observations;
    xy = [[observations.x_mm]', [observations.y_mm]'];
    plot(ax, xy(:,1), xy(:,2), '.', 'Color', [0.20 0.70 0.85], ...
        'MarkerSize', 10, 'DisplayName', 'PREPARE stationary start observations');
end


function plot_prepare_release_lines(ax, trace)

    if ~isfield(trace, 'startup') || ~isfield(trace.startup, 'prepare_observations') || ...
            isempty(trace.startup.prepare_observations)
        return;
    end
    releaseS = [trace.startup.prepare_observations.replay_t_ms] / 1000.0;
    xline(ax, releaseS, ':', 'Color', [0.20 0.70 0.85], 'HandleVisibility', 'off');
end


function [xy, tMs] = source_workspace_points(events)

    eligible = arrayfun(@(event) event.valid && event.inside_workspace, events);
    selected = events(eligible);
    xy = [[selected.x_mm]', [selected.y_mm]'];
    tMs = [events.replay_t_ms]';
end


function xy = cartesian_trace_points(segments)

    xy = zeros(0, 2);
    for i = 1:numel(segments)
        profile = segments(i).profile;
        durationS = (profile.end.t_ms - profile.start.t_ms) / 1000.0;
        local = linspace(0.0, durationS, max(2, ceil(durationS * 100) + 1));
        % Python stores [c0, ..., c5] in ascending powers. MATLAB poly() is
        % for roots, so the curve is evaluated explicitly instead.
        x = evaluate_cartesian_polynomial(profile.coefficients_x, local);
        y = evaluate_cartesian_polynomial(profile.coefficients_y, local);
        xy = [xy; [x(:), y(:)]]; %#ok<AGROW>
    end
end


function textValue = active_text(index)

    if isfinite(index), textValue = sprintf('%d', index); else, textValue = 'HOLD'; end
end


function handles = render_robot(robot, q, ax)

    before = ax.Children;
    show(robot, q, 'Parent', ax, 'PreservePlot', true, 'Frames', 'off');
    after = ax.Children;
    handles = after(~ismember(after, before));
end


function handle = render_skeleton(robot, q, ax)

    p = body_points(robot, q);
    handle = struct();
    handle.link_line = plot3(ax, p(:,1), p(:,2), p(:,3), '-', 'Color', [0.10 0.35 0.85], ...
        'LineWidth', 4, 'HandleVisibility', 'off');
    colors = lines(robot.NumBodies);
    handle.joint_markers = gobjects(robot.NumBodies + 1, 1);
    handle.joint_markers(1) = plot3(ax, p(1,1), p(1,2), p(1,3), 's', ...
        'Color', [0.10 0.10 0.10], 'MarkerFaceColor', [0.65 0.65 0.65], ...
        'MarkerSize', 8, 'HandleVisibility', 'off');
    for i = 1:robot.NumBodies
        handle.joint_markers(i+1) = plot3(ax, p(i+1,1), p(i+1,2), p(i+1,3), 'o', ...
            'Color', colors(i,:), 'MarkerFaceColor', colors(i,:), 'MarkerSize', 9, ...
            'HandleVisibility', 'off');
    end
end


function update_skeleton(handle, robot, q)

    p = body_points(robot, q);
    set(handle.link_line, 'XData', p(:,1), 'YData', p(:,2), 'ZData', p(:,3));
    for i = 1:numel(handle.joint_markers)
        set(handle.joint_markers(i), 'XData', p(i,1), 'YData', p(i,2), 'ZData', p(i,3));
    end
end


function handles = render_joint_labels(robot, q, ax)

    p = body_points(robot, q);
    handles = gobjects(robot.NumBodies + 1, 1);
    offsets = label_offsets(robot.NumBodies + 1);
    handles(1) = text(ax, p(1,1)+offsets(1,1), p(1,2)+offsets(1,2), p(1,3)+offsets(1,3), ' base', 'FontWeight', 'bold', ...
        'Color', [0.15 0.15 0.15], 'HandleVisibility', 'off');
    for i = 1:robot.NumBodies
        label = sprintf(' J%d:%s', i, robot.Bodies{i}.Joint.Name);
        handles(i+1) = text(ax, p(i+1,1)+offsets(i+1,1), p(i+1,2)+offsets(i+1,2), p(i+1,3)+offsets(i+1,3), label, ...
            'FontSize', 8, 'Color', [0.10 0.10 0.10], 'HandleVisibility', 'off');
    end
end


function update_joint_labels(handles, robot, q)

    p = body_points(robot, q);
    offsets = label_offsets(numel(handles));
    for i = 1:numel(handles)
        set(handles(i), 'Position', p(i,:) + offsets(i,:));
    end
end


function handles = render_joint_frames(robot, q, ax)

    [origins, rotations] = body_frames(robot, q);
    scale = 0.018;
    colors = [0.85 0.20 0.20; 0.15 0.60 0.20; 0.15 0.35 0.90];
    handles = gobjects(robot.NumBodies, 3);
    for bodyIndex = 1:robot.NumBodies
        for axisIndex = 1:3
            direction = rotations(:,axisIndex,bodyIndex)' * scale;
            handles(bodyIndex,axisIndex) = quiver3(ax, origins(bodyIndex,1), origins(bodyIndex,2), origins(bodyIndex,3), ...
                direction(1), direction(2), direction(3), 0, 'Color', colors(axisIndex,:), ...
                'LineWidth', 1.0, 'MaxHeadSize', 0.4, 'HandleVisibility', 'off');
        end
    end
end


function update_joint_frames(handles, robot, q)

    [origins, rotations] = body_frames(robot, q);
    scale = 0.018;
    for bodyIndex = 1:robot.NumBodies
        for axisIndex = 1:3
            direction = rotations(:,axisIndex,bodyIndex)' * scale;
            set(handles(bodyIndex,axisIndex), 'XData', origins(bodyIndex,1), ...
                'YData', origins(bodyIndex,2), 'ZData', origins(bodyIndex,3), ...
                'UData', direction(1), 'VData', direction(2), 'WData', direction(3));
        end
    end
end


function p = body_points(robot, q)

    p = zeros(robot.NumBodies + 1, 3);
    for i = 1:robot.NumBodies
        pose = getTransform(robot, q, robot.Bodies{i}.Name);
        p(i+1,:) = pose(1:3,4)';
    end
end


function points = robot_body_extent_points(robot, qTrace)
%ROBOT_BODY_EXTENT_POINTS Collect base/body origins across the whole animation.

    points = zeros((robot.NumBodies + 1) * size(qTrace,1), 3);
    cursor = 1;
    for row = 1:size(qTrace,1)
        p = body_points(robot, qTrace(row,:));
        points(cursor:cursor + size(p,1) - 1,:) = p;
        cursor = cursor + size(p,1);
    end
end


function offsets = label_offsets(count)
%LABEL_OFFSETS Keep coincident MDH origins readable instead of overprinting.

    offsets = zeros(count, 3);
    for i = 1:count
        offsets(i,:) = [0.015 * mod(i-1, 3), 0.012 * floor((i-1)/3), ...
            0.014 + 0.006 * mod(i-1, 2)];
    end
    % body1/body2 share an MDH origin.  Separate their labels deliberately so
    % a visually coincident origin is not mistaken for a missing joint.
    if count >= 3
        offsets(2,:) = [-0.030, 0.020, 0.030];
        offsets(3,:) = [0.030, -0.020, 0.014];
    end
end


function [origins, rotations] = body_frames(robot, q)

    origins = zeros(robot.NumBodies, 3);
    rotations = zeros(3, 3, robot.NumBodies);
    for i = 1:robot.NumBodies
        pose = getTransform(robot, q, robot.Bodies{i}.Name);
        origins(i,:) = pose(1:3,4)';
        rotations(:,:,i) = pose(1:3,1:3);
    end
end


function set_robot_limits(ax, points)

    points = points(all(isfinite(points),2),:);
    lower = min(points, [], 1); upper = max(points, [], 1);
    padding = max(0.15*(upper-lower), 0.02);
    xlim(ax, [lower(1)-padding(1), upper(1)+padding(1)]);
    ylim(ax, [lower(2)-padding(2), upper(2)+padding(2)]);
    zlim(ax, [lower(3)-padding(3), upper(3)+padding(3)]);
end


function value = evaluate_cartesian_polynomial(coefficients, t)

    value = coefficients(1) + coefficients(2)*t + coefficients(3)*t.^2 + ...
        coefficients(4)*t.^3 + coefficients(5)*t.^4 + coefficients(6)*t.^5;
end
