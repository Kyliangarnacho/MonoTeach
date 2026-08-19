function config = default_timed_joint_trajectory_config(robotContext)
%DEFAULT_TIMED_JOINT_TRAJECTORY_CONFIG First velocity-only timing baseline.
%
% These are planning limits for derived Stage 3.3 artifacts, not verified
% actuator limits or an execution-safety claim.  Jerk, blend, and controller
% constraints remain outside this baseline.

    if nargin < 1 || isempty(robotContext)
        robotContext = load_robot_context("legacy5");
    end
    if ~isstruct(robotContext) || ~isscalar(robotContext) || ...
            ~all(isfield(robotContext, {'dof', 'velocity_limits', ...
            'acceleration_limits'})) || robotContext.dof < 1 || ...
            ~isequal(size(robotContext.velocity_limits), [1, robotContext.dof]) || ...
            ~isequal(size(robotContext.acceleration_limits), [1, robotContext.dof]) || ...
            any(~isfinite(robotContext.velocity_limits)) || ...
            any(~isfinite(robotContext.acceleration_limits)) || ...
            any(robotContext.velocity_limits <= 0) || ...
            any(robotContext.acceleration_limits <= 0)
        error('MonoTeach:UnavailableRobotTimingLimits', ...
            'Timing defaults require finite RobotContext planning limits.');
    end
    config.max_velocity_rad_s = robotContext.velocity_limits;
    config.max_acceleration_rad_s2 = robotContext.acceleration_limits;
    config.minimum_segment_dt_s = 0.02;
    config.continuous_sample_period_s = 0.01;
    config.max_time_stretch_iterations = 8;

    % Minimum-jerk joint-envelope guard.  These are geometric diagnostics and
    % local boundary-condition damping controls; they do not alter waypoints,
    % Task Plane geometry, IK, or time allocation policy.
    config.overshoot_numerical_epsilon_rad = 1.0e-8;
    config.overshoot_guard_sample_period_s = 0.001;
    config.max_overshoot_correction_iterations = 6;
    config.overshoot_boundary_damping_factor = 0.5;
end
