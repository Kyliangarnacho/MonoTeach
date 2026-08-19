function config = default_writing_yaw_sweep_config()
%DEFAULT_WRITING_YAW_SWEEP_CONFIG Coarse vertical Task Plane yaw-height scan.
%
% Each pose uses Rz_base(yaw) * R0. Only downward centre-height samples are
% included because the default 0.255 m plane already spans below the Home
% body5-origin height; this is engineering context, not a reachability proof.

    config.yaw_deg_values = -60:10:60;
    config.center_z_values_m = 0.195:0.010:0.255;
    config.random_seed = 7331;
    config.random_seed_count = 1;
    config.solver_algorithm = 'BFGSGradientProjection';
    config.allow_random_restart = false;
end
