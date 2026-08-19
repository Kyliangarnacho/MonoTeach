function config = default_writing_full_plane_scan_config()
%DEFAULT_WRITING_FULL_PLANE_SCAN_CONFIG Coarse full Task Plane point scan.
%
% This is a temporary Step 8 candidate only.  It does not change the formal
% default Task Plane or select a permanent writing placement.

    config.workspace_x_values_mm = linspace(0.0, 190.0, 9);
    config.workspace_y_values_mm = linspace(0.0, 290.0, 13);
    config.traversal_order = 'workspace_x_then_y';

    config.candidate_center_m = [0.105, 0.060, 0.195];
    config.candidate_yaw_deg = -60.0;
    config.normal_sign = +1;

    config.random_seed = 7331;
    config.random_seed_count = 1;
    config.solver_algorithm = 'BFGSGradientProjection';
    config.allow_random_restart = false;
end
