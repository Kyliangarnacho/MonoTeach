function demo = demo_continuous_ik(jsonPath)
%DEMO_CONTINUOUS_IK Static Stage 2.3 WorkspaceTrajectory -> IK inspection.
%
% Input:
%   jsonPath - one real Stage 2.3 WorkspaceTrajectory JSON artifact
%
% Output:
%   demo - derived pipeline artifacts and figure handles for inspection
%
% This static demo intentionally performs no robot animation, time
% resampling, velocity/acceleration calculation, or execution command.

    if nargin ~= 1 || ~(ischar(jsonPath) || ...
            (isstring(jsonPath) && isscalar(jsonPath)))
        error( ...
            'MonoTeach:InvalidContinuousIKDemoInput', ...
            'demo_continuous_ik requires one WorkspaceTrajectory JSON path.');
    end

    jsonPath = char(string(jsonPath));

    if ~isfile(jsonPath)
        error( ...
            'MonoTeach:MissingWorkspaceTrajectory', ...
            'WorkspaceTrajectory JSON does not exist: %s', ...
            jsonPath);
    end

    stage3Dir = fileparts(fileparts(mfilename('fullpath')));
    repoRoot = fileparts(stage3Dir);
    stage1Dir = fullfile(repoRoot, 'stage1');
    addpath(genpath(stage3Dir));
    addpath(stage1Dir);

    % Desired task XYZ comes from the unchanged Stage 2.3 workspace samples
    % after the established Stage 3.1 workspace -> task-plane -> robot-base
    % transform. The pre-IK builder preserves its barriers before any IK.
    workspaceTrajectory = load_workspace_trajectory(jsonPath);
    taskPlaneConfig = default_task_plane_config();
    taskTrajectory = workspace_to_task_trajectory( ...
        workspaceTrajectory, ...
        taskPlaneConfig);
    preIkSegmentSet = build_preik_segments(taskTrajectory);

    robot = build_legacy_robot();
    ikConfig = default_ik_config();
    anchorResult = solve_anchor_ik(robot, taskPlaneConfig, ikConfig);
    ikResultSet = solve_ik_segments( ...
        robot, ...
        preIkSegmentSet, ...
        anchorResult, ...
        ikConfig);
    continuity = summarize_ik_continuity(ikResultSet);

    figureDesiredFk = draw_desired_and_fk_paths( ...
        ikResultSet, ...
        taskPlaneConfig);
    figureJointWaypoints = draw_joint_waypoints(ikResultSet);
    print_summary( ...
        jsonPath, ...
        preIkSegmentSet, ...
        ikResultSet, ...
        continuity);

    demo = struct();
    demo.json_path = jsonPath;
    demo.workspace_trajectory = workspaceTrajectory;
    demo.task_trajectory = taskTrajectory;
    demo.preik_segment_set = preIkSegmentSet;
    demo.anchor_result = anchorResult;
    demo.ik_result_set = ikResultSet;
    demo.continuity = continuity;
    demo.figure_desired_fk = figureDesiredFk;
    demo.figure_joint_waypoints = figureJointWaypoints;
end


function figureHandle = draw_desired_and_fk_paths(ikResultSet, taskPlaneConfig)

    figureHandle = figure( ...
        'Name', 'MonoTeach Stage 3.2C Desired and FK Paths');
    ax = axes(figureHandle);
    hold(ax, 'on');
    colors = lines(max(numel(ikResultSet.segments), 1));
    legendHandles = gobjects(1, 0);
    plottedXYZ = zeros(0, 3);

    % Desired points are the robot-base targets preserved in each canonical
    % result. FK points are getTransform outputs stored after each accepted IK
    % solve. Plotting both is the IK -> FK closed-loop geometry back-check.
    for i = 1:numel(ikResultSet.segments)
        successSegment = ikResultSet.segments(i);
        pointResults = successSegment.results;
        desiredXYZ = vertcat(pointResults.target_xyz_m);
        fkXYZ = vertcat(pointResults.fk_xyz_m);
        plottedXYZ = [plottedXYZ; desiredXYZ; fkXYZ]; %#ok<AGROW>

        desiredHandle = plot3( ...
            ax, ...
            desiredXYZ(:, 1), ...
            desiredXYZ(:, 2), ...
            desiredXYZ(:, 3), ...
            'o-', ...
            'Color', colors(i, :), ...
            'MarkerFaceColor', colors(i, :), ...
            'LineWidth', 1.8, ...
            'DisplayName', sprintf('Desired success segment %d', i));
        fkHandle = plot3( ...
            ax, ...
            fkXYZ(:, 1), ...
            fkXYZ(:, 2), ...
            fkXYZ(:, 3), ...
            's--', ...
            'Color', colors(i, :), ...
            'MarkerFaceColor', 'w', ...
            'LineWidth', 1.4, ...
            'DisplayName', sprintf('FK success segment %d', i));
        legendHandles = [legendHandles, desiredHandle, fkHandle]; %#ok<AGROW>
    end

    if ~isempty(ikResultSet.failures)
        % Failures have target evidence but no accepted q/FK point. Mark their
        % desired location only; never bridge it with a fabricated path line.
        failureXYZ = vertcat(ikResultSet.failures.target_xyz_m);
        plottedXYZ = [plottedXYZ; failureXYZ]; %#ok<AGROW>
        failureHandle = plot3( ...
            ax, ...
            failureXYZ(:, 1), ...
            failureXYZ(:, 2), ...
            failureXYZ(:, 3), ...
            'rx', ...
            'MarkerSize', 9, ...
            'LineWidth', 1.8, ...
            'DisplayName', 'IK failure target (no FK path)');
        legendHandles = [legendHandles, failureHandle]; %#ok<AGROW>
    end

    anchor = taskPlaneConfig.robot_anchor_m;
    plottedXYZ(end + 1, :) = anchor;
    anchorHandle = plot3( ...
        ax, ...
        anchor(1), ...
        anchor(2), ...
        anchor(3), ...
        'kp', ...
        'MarkerSize', 11, ...
        'LineWidth', 1.5, ...
        'DisplayName', 'Task-plane anchor');
    legendHandles = [legendHandles, anchorHandle]; %#ok<AGROW>

    set_base_path_limits(ax, plottedXYZ);
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'Robot base X (m)');
    ylabel(ax, 'Robot base Y (m)');
    zlabel(ax, 'Robot base Z (m)');
    title(ax, 'Stage 3.2C Desired Task XYZ and FK Recovered XYZ');
    % The configured task plane lies at nearly constant base Y. Viewing along
    % base Y exposes its X-Z motion while preserving robot-base metre axes;
    % this is a static geometry view, not an animation or time projection.
    view(ax, [0, 0]);

    if ~isempty(legendHandles)
        legend(ax, legendHandles, 'Location', 'best');
    end
end


function set_base_path_limits(ax, pointsM)

    lower = min(pointsM, [], 1);
    upper = max(pointsM, [], 1);
    span = upper - lower;
    padding = max(0.15 * span, 0.003);

    xlim(ax, [lower(1) - padding(1), upper(1) + padding(1)]);
    ylim(ax, [lower(2) - padding(2), upper(2) + padding(2)]);
    zlim(ax, [lower(3) - padding(3), upper(3) + padding(3)]);
end


function figureHandle = draw_joint_waypoints(ikResultSet)

    figureHandle = figure( ...
        'Name', 'MonoTeach Stage 3.2C Joint Waypoints');
    ax = axes(figureHandle);
    hold(ax, 'on');
    jointColors = lines(5);
    jointHandles = gobjects(1, 5);

    for i = 1:numel(ikResultSet.segments)
        successSegment = ikResultSet.segments(i);
        sourceIndices = successSegment.source_indices;
        qRad = successSegment.q_rad;

        % One plot call per successful subsegment intentionally leaves visual
        % gaps across Stage 3.1 barriers and IK failures. source_index is used
        % only as waypoint provenance, not as a uniformly sampled time axis.
        for jointIndex = 1:5
            lineHandle = plot( ...
                ax, ...
                sourceIndices, ...
                qRad(:, jointIndex), ...
                'o-', ...
                'Color', jointColors(jointIndex, :), ...
                'MarkerFaceColor', jointColors(jointIndex, :), ...
                'LineWidth', 1.5, ...
                'HandleVisibility', 'off');

            if i == 1
                jointHandles(jointIndex) = lineHandle;
            end
        end
    end

    for i = 1:numel(ikResultSet.failures)
        % A failure is an explicit waypoint barrier, not an interpolated q.
        xline( ...
            ax, ...
            ikResultSet.failures(i).source_index, ...
            'r--', ...
            'HandleVisibility', 'off');
    end

    grid(ax, 'on');
    xlabel(ax, 'Stage 3 source index (waypoint provenance, not time)');
    ylabel(ax, 'Joint position (rad)');
    title(ax, 'Stage 3.2C q1-q5 Joint Waypoints by IK-Success Subsegment');

    visibleJointHandles = jointHandles(isgraphics(jointHandles));
    if ~isempty(visibleJointHandles)
        legend( ...
            ax, ...
            visibleJointHandles, ...
            {'q1', 'q2', 'q3', 'q4', 'q5'}, ...
            'Location', 'best');
    end
end


function print_summary(jsonPath, preIkSegmentSet, ikResultSet, continuity)

    fprintf('============================================\n');
    fprintf('MonoTeach Stage 3.2C Continuous IK Static Demo\n');
    fprintf('============================================\n');
    fprintf('WorkspaceTrajectory : %s\n', jsonPath);
    fprintf('Pre-IK segments     : %d\n', ...
        numel(preIkSegmentSet.segments));
    fprintf('IK-success segments : %d\n', ...
        numel(ikResultSet.segments));
    fprintf('Success / failures  : %d / %d\n', ...
        continuity.success_point_count, ...
        continuity.failure_count);
    fprintf('IK success rate     : %.6f\n', continuity.ik_success_rate);
    fprintf('Mean FK error [m]   : %.9e\n', ...
        continuity.mean_fk_position_error_m);
    fprintf('Max FK error [m]    : %.9e\n', ...
        continuity.max_fk_position_error_m);
    fprintf('Per-joint max |dq|  : %s rad\n', ...
        mat2str(continuity.per_joint_max_abs_delta_q, 8));
    fprintf('Overall max step    : %.9e rad\n', ...
        continuity.overall_max_joint_step);
    fprintf('Largest jump        : source %s, joint %s\n', ...
        scalar_or_empty_text(continuity.largest_jump_source_index), ...
        scalar_or_empty_text(continuity.largest_jump_joint_index));
    fprintf('Minimum limit margin: %.9e rad', ...
        continuity.minimum_joint_limit_margin);

    if ~isempty(continuity.closest_limit_joint_index)
        fprintf( ...
            ' (joint %d, source %d)', ...
            continuity.closest_limit_joint_index, ...
            continuity.closest_limit_source_index);
    end

    fprintf('\n');

    if isempty(ikResultSet.failures)
        fprintf('Failures            : 0 (none repaired or interpolated)\n');
    else
        fprintf('Failure evidence:\n');

        for i = 1:numel(ikResultSet.failures)
            failure = ikResultSet.failures(i);
            fprintf( ...
                '  source %d, t_ms %.3f, reason %s\n', ...
                failure.source_index, ...
                failure.t_ms, ...
                char(string(failure.reason)));
        end
    end

    fprintf('============================================\n');
end


function textValue = scalar_or_empty_text(value)

    if isempty(value)
        textValue = 'none';
    else
        textValue = num2str(value);
    end
end
