function config = default_writing_xy_sweep_config()
%DEFAULT_WRITING_XY_SWEEP_CONFIG Coarse local XY scan around Step 6 candidate.
%
% The fixed orientation and centre height are the closest formal-rejection
% result from the yaw-height sweep.  This scan changes only the whole Task
% Plane's Base X/Y translation; it is not a full-plane trajectory test.

    config.x_values_m = 0.070:0.005:0.130;
    config.y_values_m = 0.010:0.005:0.070;

    config.fixed_yaw_deg = -60.0;
    config.fixed_center_z_m = 0.195;
    config.normal_sign = +1;

    config.random_seed = 7331;
    config.random_seed_count = 1;
    config.solver_algorithm = 'BFGSGradientProjection';
    config.allow_random_restart = false;
end
