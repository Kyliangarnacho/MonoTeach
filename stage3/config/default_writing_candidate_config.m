function config = default_writing_candidate_config()
%DEFAULT_WRITING_CANDIDATE_CONFIG Stage 3.3 working writing placement only.
%
% These values are deliberately separate from default_task_plane_config.m.
% They are an experimentally selected Stage 3.3 candidate, not a change to
% the frozen Stage 3.1/3.2 default placement or mapping scale.

    config.center_m = [0.105, 0.060, 0.195];
    config.yaw_deg = -60.0;
    config.normal_sign = +1;
    config.scale_m_per_mm = 0.00020;

    config.source_workspace_width_mm = 190.0;
    config.source_workspace_height_mm = 290.0;
end
