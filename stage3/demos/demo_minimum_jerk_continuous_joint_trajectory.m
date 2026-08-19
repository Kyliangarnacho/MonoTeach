function demo = demo_minimum_jerk_continuous_joint_trajectory(jsonPath, timingConfig)
%DEMO_MINIMUM_JERK_CONTINUOUS_JOINT_TRAJECTORY Plot longest real segment only.

    if nargin < 2 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config();
    end
    if nargin < 1, jsonPath = []; end
    if isempty(jsonPath)
        run = run_real_resampled_task_path_ik_ab();
    else
        run = run_real_resampled_task_path_ik_ab(jsonPath);
    end
    timed = time_parameterize_ik_success_segments( ...
        run.resampled_writing_result_set, timingConfig);
    [~, selectedInputIndex] = max(arrayfun( ...
        @(segment) numel(segment.waypoints), timed.segments));
    selectedTimed = timed;
    selectedTimed.segments = timed.segments(selectedInputIndex);
    continuous = generate_minimum_jerk_continuous_joint_trajectory( ...
        selectedTimed, timingConfig);
    segment = continuous.segments(1);

    fprintf(['Minimum-jerk segment %d: %d waypoints, %d samples, initial ' ...
        'duration %.6f s, final duration %.6f s, stretched=%d\n'], ...
        segment.segment_index, size(segment.waypoint_q_rad, 1), ...
        numel(segment.t_s), segment.initial_duration_s, segment.duration_s, ...
        segment.was_time_stretched);
    fprintf('Peak velocity [rad/s]: %s\n', ...
        mat2str(segment.max_observed_abs_velocity_rad_s, 8));
    fprintf('Peak acceleration [rad/s^2]: %s\n', ...
        mat2str(segment.max_observed_abs_acceleration_rad_s2, 8));
    fprintf('Peak jerk [rad/s^3]: %s\n', ...
        mat2str(segment.max_observed_abs_jerk_rad_s3, 8));
    fprintf('Initial limit ratios [velocity acceleration]: [%.6f %.6f]\n', ...
        segment.initial_velocity_limit_ratio, ...
        segment.initial_acceleration_limit_ratio);
    fprintf('All limits satisfied: %d\n', segment.limits_satisfied);

    figure('Name', 'Minimum-jerk ContinuousJointTrajectory V1');
    tiledlayout(3, 1);
    nexttile;
    plot(segment.t_s, segment.q_rad, 'LineWidth', 1.2); hold on;
    plot(segment.waypoint_t_s, segment.waypoint_q_rad, 'o');
    grid on; xlabel('Local segment time [s]'); ylabel('q [rad]');
    title(sprintf('IK-success segment %d', segment.segment_index));
    nexttile;
    plot(segment.t_s, segment.qd_rad_s, 'LineWidth', 1.2); grid on;
    xlabel('Local segment time [s]'); ylabel('qdot [rad/s]');
    nexttile;
    plot(segment.t_s, segment.qdd_rad_s2, 'LineWidth', 1.2); grid on;
    xlabel('Local segment time [s]'); ylabel('qddot [rad/s^2]');

    demo = struct('source_run', run, 'timed_trajectory', timed, ...
        'selected_input_segment_index', selectedInputIndex, ...
        'continuous_trajectory', continuous, 'selected_segment', segment);
end
