function demo = demo_timed_joint_trajectory_baseline(jsonPath, timingConfig)
%DEMO_TIMED_JOINT_TRAJECTORY_BASELINE Plot one real Writing success segment.
%
% Uses the frozen 2 mm resampled real-triangle pipeline, then derives timing
% only.  The demo does not alter its IK result set or execute the robot.

    if nargin < 2 || isempty(timingConfig)
        timingConfig = default_timed_joint_trajectory_config();
    end
    if nargin < 1, jsonPath = []; end
    if isempty(jsonPath)
        run = run_real_resampled_task_path_ik_ab();
    else
        run = run_real_resampled_task_path_ik_ab(jsonPath);
    end
    timedTrajectory = time_parameterize_ik_success_segments( ...
        run.resampled_writing_result_set, timingConfig);
    waypointCounts = arrayfun(@(segment) numel(segment.waypoints), ...
        timedTrajectory.segments);
    [~, selectedSegmentIndex] = max(waypointCounts);
    segment = timedTrajectory.segments(selectedSegmentIndex);
    waypoints = segment.waypoints;
    t = [waypoints.t_s]';
    q = vertcat(waypoints.q_rad);
    dt = [waypoints.dt_s]';
    velocity = vertcat(waypoints.step_velocity_rad_s);

    figure('Name', 'TimedJointTrajectory velocity-limited baseline');
    tiledlayout(3, 1);
    nexttile;
    plot(t, q, 'LineWidth', 1.25);
    grid on;
    xlabel('Local segment time [s]'); ylabel('q [rad]');
    legend({'q1', 'q2', 'q3', 'q4', 'q5'}, 'Location', 'best');
    title(sprintf('IK-success segment %d (%d waypoints)', ...
        selectedSegmentIndex, numel(waypoints)));
    nexttile;
    bar(t, dt);
    grid on;
    xlabel('Local segment time [s]'); ylabel('dt [s]');
    nexttile;
    plot(t, velocity, 'LineWidth', 1.25); hold on;
    plot(t, repmat(timingConfig.max_velocity_rad_s, numel(t), 1), '--');
    plot(t, -repmat(timingConfig.max_velocity_rad_s, numel(t), 1), '--');
    grid on;
    xlabel('Local segment time [s]'); ylabel('Step velocity [rad/s]');

    if ~all([waypoints.within_velocity_limits])
        error('MonoTeach:TimedJointVelocityLimitViolation', ...
            'Derived demo segment violates a configured velocity limit.');
    end
    demo = struct('source_run', run, 'timed_trajectory', timedTrajectory, ...
        'selected_segment_index', selectedSegmentIndex, ...
        'selected_segment', segment, ...
        'max_observed_abs_velocity_rad_s', ...
        timedTrajectory.summary.max_observed_abs_velocity_rad_s);
end
