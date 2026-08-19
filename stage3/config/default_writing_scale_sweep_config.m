function config = default_writing_scale_sweep_config()
%DEFAULT_WRITING_SCALE_SWEEP_CONFIG Coarse uniform writing-scale plan.
%
% Source Workspace dimensions remain 190-by-290 mm.  Only the temporary
% Workspace-to-Robot uniform scale changes; no formal default is selected.

    config.scale_values_m_per_mm = [ ...
        0.00010, 0.00015, 0.00020, 0.00025, 0.00030, ...
        0.00035, 0.00040, 0.00045, 0.00050];
    config.coarse_workspace_x_values_mm = linspace(0.0, 190.0, 5);
    config.coarse_workspace_y_values_mm = linspace(0.0, 290.0, 7);
    config.verification_workspace_x_values_mm = linspace(0.0, 190.0, 9);
    config.verification_workspace_y_values_mm = linspace(0.0, 290.0, 13);

    config.candidate_center_m = [0.105, 0.060, 0.195];
    config.candidate_yaw_deg = -60.0;
    config.normal_sign = +1;
    config.traversal_order = 'workspace_x_then_y';

    config.random_seed = 7331;
    config.random_seed_count = 1;
    config.solver_algorithm = 'BFGSGradientProjection';
    config.allow_random_restart = false;
end
