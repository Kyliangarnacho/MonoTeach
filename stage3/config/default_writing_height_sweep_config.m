function config = default_writing_height_sweep_config()
%DEFAULT_WRITING_HEIGHT_SWEEP_CONFIG Coarse, deterministic anchor-Z scan.
%
% This is intentionally a small local sweep around the default 0.255 m
% placement. It does not alter default_task_plane_config.m.

    config.anchor_z_values_m = 0.195:0.010:0.275;
    config.random_seed = 7331;
    config.random_seed_count = 3;
    config.solver_algorithm = 'BFGSGradientProjection';
    config.allow_random_restart = false;
end
