function demo = demo_real_trajectory_writing_ik(jsonPath)
%DEMO_REAL_TRAJECTORY_WRITING_IK Static desired/FK and joint-result inspection.
%
% Successful subsegments are plotted separately. Failure targets are marked,
% and q traces are deliberately broken at every failure barrier.

    if nargin < 1
        run = run_real_trajectory_writing_ik();
    else
        run = run_real_trajectory_writing_ik(jsonPath);
    end

    figureDesiredFk = draw_desired_and_fk( ...
        run.writing_rescued_result_set, run.candidate_task_plane_config);
    figureJointQ = draw_joint_waypoints(run.writing_rescued_result_set);
    print_summary(run);

    demo = run;
    demo.figure_desired_fk = figureDesiredFk;
    demo.figure_joint_waypoints = figureJointQ;
end


function figureHandle = draw_desired_and_fk(writingResultSet, taskPlaneConfig)

    figureHandle = figure('Name', ...
        'MonoTeach Stage 3.3 Writing IK Desired and FK Paths');
    ax = axes(figureHandle);
    hold(ax, 'on');
    colors = lines(max(numel(writingResultSet.segments), 1));
    legendHandles = gobjects(1, 0);
    plottedXYZ = zeros(0, 3);
    for segmentIndex = 1:numel(writingResultSet.segments)
        segment = writingResultSet.segments(segmentIndex);
        results = segment.results;
        desiredXYZ = vertcat(results.target_xyz_m);
        fkXYZ = vertcat(results.fk_xyz_m);
        plottedXYZ = [plottedXYZ; desiredXYZ; fkXYZ]; %#ok<AGROW>
        desiredHandle = plot3( ...
            ax, desiredXYZ(:, 1), desiredXYZ(:, 2), desiredXYZ(:, 3), ...
            'o-', 'Color', colors(segmentIndex, :), ...
            'MarkerFaceColor', colors(segmentIndex, :), ...
            'LineWidth', 1.5, ...
            'DisplayName', sprintf('Desired success segment %d', segmentIndex));
        fkHandle = plot3( ...
            ax, fkXYZ(:, 1), fkXYZ(:, 2), fkXYZ(:, 3), ...
            's--', 'Color', colors(segmentIndex, :), ...
            'MarkerFaceColor', 'w', 'LineWidth', 1.2, ...
            'DisplayName', sprintf('FK success segment %d', segmentIndex));
        legendHandles = [legendHandles, desiredHandle, fkHandle]; %#ok<AGROW>
    end
    if ~isempty(writingResultSet.failures)
        failureXYZ = vertcat(writingResultSet.failures.target_xyz_m);
        plottedXYZ = [plottedXYZ; failureXYZ]; %#ok<AGROW>
        failureHandle = plot3( ...
            ax, failureXYZ(:, 1), failureXYZ(:, 2), failureXYZ(:, 3), ...
            'rx', 'MarkerSize', 9, 'LineWidth', 1.5, ...
            'DisplayName', 'Writing IK failure target');
        legendHandles = [legendHandles, failureHandle]; %#ok<AGROW>
    end
    anchor = taskPlaneConfig.robot_anchor_m;
    plottedXYZ(end + 1, :) = anchor;
    plot3(ax, anchor(1), anchor(2), anchor(3), 'kp', ...
        'MarkerSize', 10, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    set_limits(ax, plottedXYZ);
    axis(ax, 'equal');
    grid(ax, 'on');
    xlabel(ax, 'Robot base X (m)');
    ylabel(ax, 'Robot base Y (m)');
    zlabel(ax, 'Robot base Z (m)');
    title(ax, 'Writing IK desired XYZ and accepted FK XYZ');
    view(ax, [0, 0]);
    if ~isempty(legendHandles)
        legend(ax, legendHandles, 'Location', 'best');
    end
end


function figureHandle = draw_joint_waypoints(writingResultSet)

    figureHandle = figure('Name', ...
        'MonoTeach Stage 3.3 Writing IK Joint Waypoints');
    ax = axes(figureHandle);
    hold(ax, 'on');
    colors = lines(5);
    handles = gobjects(1, 5);
    for segmentIndex = 1:numel(writingResultSet.segments)
        segment = writingResultSet.segments(segmentIndex);
        for jointIndex = 1:5
            handle = plot( ...
                ax, segment.source_indices, segment.q_rad(:, jointIndex), ...
                'o-', 'Color', colors(jointIndex, :), ...
                'MarkerFaceColor', colors(jointIndex, :), ...
                'LineWidth', 1.3, 'HandleVisibility', 'off');
            if segmentIndex == 1
                handles(jointIndex) = handle;
            end
        end
    end
    for failureIndex = 1:numel(writingResultSet.failures)
        xline(ax, writingResultSet.failures(failureIndex).source_index, ...
            'r--', 'HandleVisibility', 'off');
    end
    grid(ax, 'on');
    xlabel(ax, 'Stage 3 source index (not time)');
    ylabel(ax, 'Joint position (rad)');
    title(ax, 'Writing IK q1-q5; lines break at failures');
    visibleHandles = handles(isgraphics(handles));
    if ~isempty(visibleHandles)
        legend(ax, visibleHandles, {'q1', 'q2', 'q3', 'q4', 'q5'}, ...
            'Location', 'best');
    end
end


function set_limits(ax, pointsM)

    lower = min(pointsM, [], 1);
    upper = max(pointsM, [], 1);
    padding = max(0.15 * (upper - lower), 0.003);
    xlim(ax, [lower(1) - padding(1), upper(1) + padding(1)]);
    ylim(ax, [lower(2) - padding(2), upper(2) + padding(2)]);
    zlim(ax, [lower(3) - padding(3), upper(3) + padding(3)]);
end


function print_summary(run)

    forwardWriting = run.writing_summary;
    writing = run.writing_rescued_summary;
    oldWriting = run.writing_old_policy_summary;
    position = run.position_only_summary;
    fprintf('Recovery A/B old/new success: %d/%d -> %d/%d, rates %.6f -> %.6f\n', ...
        oldWriting.success_point_count, oldWriting.failure_count, ...
        forwardWriting.success_point_count, forwardWriting.failure_count, ...
        oldWriting.ik_success_rate, forwardWriting.ik_success_rate);
    fprintf('Forward/backward rescue success: %d/%d -> %d/%d, rates %.6f -> %.6f\n', ...
        forwardWriting.success_point_count, forwardWriting.failure_count, ...
        writing.success_point_count, writing.failure_count, ...
        forwardWriting.ik_success_rate, writing.ik_success_rate);
    disp('New Writing accepted-seed provenance:');
    disp(writing.accepted_seed_provenance_distribution);
    disp('New Writing failure reasons:');
    disp(writing.failure_reason_distribution);
    fprintf('Writing IK, success/failure: %d/%d, rate %.6f\n', ...
        writing.success_point_count, writing.failure_count, ...
        writing.ik_success_rate);
    fprintf('Writing FK pos mean/max [m]: %.9g / %.9g\n', ...
        writing.mean_fk_position_error_m, writing.max_fk_position_error_m);
    fprintf('Writing direction mean/max [deg]: %.9g / %.9g\n', ...
        writing.mean_tool_direction_error_deg, ...
        writing.max_tool_direction_error_deg);
    fprintf('Position-Only success rate: %.6f\n', position.ik_success_rate);
    if ~isempty(writing.failure_source_indices)
        fprintf('Writing failure source indices: %s\n', ...
            mat2str(writing.failure_source_indices));
    end
end
