function demo = demo_minimum_jerk_overshoot_diagnostic(jsonPath, timingConfig)
%DEMO_MINIMUM_JERK_OVERSHOOT_DIAGNOSTIC Longest real segment, three methods.

    if nargin < 2 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config();
    end
    if nargin < 1, jsonPath = []; end
    if isempty(jsonPath)
        run = run_real_resampled_task_path_ik_ab();
    else
        run = run_real_resampled_task_path_ik_ab(jsonPath);
    end
    robot = build_legacy_robot();
    timed = time_parameterize_ik_success_segments( ...
        run.resampled_writing_result_set, timingConfig);
    [~, selectedInputIndex] = max(arrayfun( ...
        @(segment) numel(segment.waypoints), timed.segments));
    selectedTimed = timed;
    selectedTimed.segments = timed.segments(selectedInputIndex);
    patched = generate_minimum_jerk_continuous_joint_trajectory( ...
        selectedTimed, timingConfig);
    posture = default_writing_posture_config();
    diagnostic = diagnose_minimum_jerk_overshoot_methods(robot, patched, ...
        run.resampled_writing_result_set, ...
        run.raw_frame_driven_baseline.candidate_task_plane_config, ...
        run.raw_frame_driven_baseline.candidate_config, posture);

    before = diagnostic.plain_minimum_jerk_before_patch.metrics;
    after = diagnostic.patched_minimum_jerk.metrics;
    fprintf(['Overshoot before/after: %.9g / %.9g rad; Cartesian mean/max ' ...
        'before %.9g / %.9g m, after %.9g / %.9g m\n'], ...
        before.max_joint_overshoot_rad, after.max_joint_overshoot_rad, ...
        before.mean_cartesian_deviation_m, before.max_cartesian_deviation_m, ...
        after.mean_cartesian_deviation_m, after.max_cartesian_deviation_m);
    for index = 1:numel(diagnostic.comparison_records)
        record = diagnostic.comparison_records(index);
        fprintf(['%s: overshoot %.9g rad, Cartesian mean/max %.9g / %.9g m, ' ...
            'tool-Z max %.6f deg\n'], record.method, ...
            record.max_joint_overshoot_rad, record.mean_cartesian_deviation_m, ...
            record.max_cartesian_deviation_m, record.max_tool_direction_error_deg);
    end

    reference = diagnostic.patched_minimum_jerk.validation.segments(1).reference_xyz_m;
    cubicFK = diagnostic.cubic.validation.segments(1).fk_xyz_m;
    quinticFK = diagnostic.quintic.validation.segments(1).fk_xyz_m;
    patchedFK = diagnostic.patched_minimum_jerk.validation.segments(1).fk_xyz_m;
    figure('Name', 'Writing FK path diagnostic: patched minimum jerk, cubic, quintic');
    plot3(reference(:,1), reference(:,2), reference(:,3), 'k--', 'LineWidth', 1.5); hold on;
    plot3(patchedFK(:,1), patchedFK(:,2), patchedFK(:,3), 'b-', 'LineWidth', 1.2);
    plot3(cubicFK(:,1), cubicFK(:,2), cubicFK(:,3), 'r-', 'LineWidth', 1.1);
    plot3(quinticFK(:,1), quinticFK(:,2), quinticFK(:,3), 'g-', 'LineWidth', 1.1);
    grid on; axis equal;
    xlabel('Base X [m]'); ylabel('Base Y [m]'); zlabel('Base Z [m]');
    legend({'Linear Writing reference', 'Patched minimum jerk FK', ...
        'Cubic FK', 'Quintic FK'}, 'Location', 'best');
    title(sprintf('IK-success segment %d', patched.segments(1).segment_index));

    demo = struct('source_run', run, 'timed_trajectory', timed, ...
        'selected_input_segment_index', selectedInputIndex, ...
        'diagnostic', diagnostic);
end
