function run = run_real_constrained_path_gap_recovery(jsonPath)
%RUN_REAL_CONSTRAINED_PATH_GAP_RECOVERY Recover internal real Writing gaps.
%
% The frozen 2 mm Writing pipeline runs unchanged first.  This runner then
% derives RecoveryWindows from its strict evidence and performs only local,
% deterministic orientation-tolerance relaxation through RobotContext.

    if nargin < 1, jsonPath = []; end
    if isempty(jsonPath)
        baseline = run_real_resampled_task_path_ik_ab();
    else
        baseline = run_real_resampled_task_path_ik_ab(jsonPath);
    end
    baselineBefore = baseline;
    robotContext = load_robot_context('legacy5');
    postureConfig = default_writing_posture_config();
    recoveryConfig = default_constrained_path_gap_recovery_config(postureConfig);
    strictWriting = baseline.resampled_writing_result_set;
    recoveryWindows = build_constrained_path_recovery_windows( ...
        baseline.resampled_preik_adapter, strictWriting);
    recovery = recover_constrained_path_gaps(robotContext, recoveryWindows, ...
        postureConfig, recoveryConfig, ...
        baseline.resampled_position_only_seed_lookup);
    execution = derive_recovered_writing_execution( ...
        baseline.resampled_preik_adapter, strictWriting, recovery);

    run = struct();
    run.artifact_type = 'RealConstrainedPathGapRecoveryRun';
    run.robot_context = robotContext;
    run.baseline = baseline;
    run.strict_writing_result_set = strictWriting;
    run.recovery_config = recoveryConfig;
    run.recovery_windows = recoveryWindows;
    run.recovery = recovery;
    run.recovered_execution = execution;
    run.summary = build_summary(execution, recovery);
    run.input_baseline_unchanged = isequaln(baseline, baselineBefore);
    if ~run.input_baseline_unchanged
        error('MonoTeach:ConstrainedGapRecoveryMutatedBaseline', ...
            'Gap recovery must not modify the frozen Writing baseline.');
    end
end


function summary = build_summary(execution, recovery)

    summary = execution.summary;
    summary.recovery_window_count = recovery.summary.window_count;
    summary.incomplete_gap_count = recovery.summary.incomplete_gap_count;
    summary.fully_recovered_gap_count = ...
        recovery.summary.fully_reconnected_gap_count;
    summary.recovered_point_count = recovery.summary.recovered_point_count;
    summary.remaining_failure_count = recovery.summary.remaining_failed_point_count;
end
