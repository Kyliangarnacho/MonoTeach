function config = default_replay_execution_config(robotContext)
%DEFAULT_REPLAY_EXECUTION_CONFIG Conservative controls for Stage 3.4 replay.
%
% execution_time_scale relates two different clocks:
%   source duration [s] * execution_time_scale = q(t) duration [s].
% Keeping it one global factor preserves shared qd/qdd boundaries after the
% source-time derivatives are converted to execution time.  It is deliberately
% a user-visible baseline parameter, not an optimizer or an actuator rating.

    if nargin < 1 || isempty(robotContext)
        robotContext = load_robot_context('legacy5');
    end
    config.execution_time_scale = 1.0;
    config.controller_period_s = 0.01;       % 100 Hz virtual controller
    % The derived replay trace may reserve a PREPARE interval for a smooth
    % Home-to-Start move.  MATLAB reads that duration from the trace so the
    % virtual source cannot begin formal tracking early.
    config.enable_start_synchronization = true;
    % Rendering selects display frames from the completed 100 Hz trace only.
    % It never changes q(t), the FIFO, or any virtual execution clock.
    config.render_rate_hz = 25.0;
    config.maximum_simulation_s = 60.0;
    config.ik_position_tolerance_m = 1.0e-4;
    config.max_velocity_rad_s = robotContext.velocity_limits;
    config.max_acceleration_rad_s2 = robotContext.acceleration_limits;
    config.limit_epsilon = 1.0e-9;
end
