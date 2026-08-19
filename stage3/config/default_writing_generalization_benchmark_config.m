function config = default_writing_generalization_benchmark_config()
%DEFAULT_WRITING_GENERALIZATION_BENCHMARK_CONFIG Frozen Stage 3.3 gate setup.
%
% This is intentionally a snapshot of the accepted working candidate, not a
% trajectory-specific tuning surface.

    config.candidate_config = default_writing_candidate_config();
    config.resampling_config = default_task_resampling_config();
    config.ik_config = default_ik_config();
    config.writing_posture_config = default_writing_posture_config();
    config.recovery_policy = ...
        default_writing_recovery_policy('position_only_recovery');
    config.workspace_width_mm = 190.0;
    config.workspace_height_mm = 290.0;
    config.use_backward_rescue = false;
end
