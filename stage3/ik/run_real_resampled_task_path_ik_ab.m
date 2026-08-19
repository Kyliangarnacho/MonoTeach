function run = run_real_resampled_task_path_ik_ab(jsonPath, resamplingConfig)
%RUN_REAL_RESAMPLED_TASK_PATH_IK_AB Compare raw and 2 mm derived task paths.
%
% The existing 91-point pipeline is invoked unchanged as the frame-driven
% baseline.  Only the independently derived resampled adapter is passed to
% the frozen Position-Only and independent Writing solvers.

    if nargin < 2 || isempty(resamplingConfig)
        resamplingConfig = default_task_resampling_config();
    end
    if nargin < 1, jsonPath = []; end
    if isempty(jsonPath)
        raw = run_real_trajectory_writing_ik();
    else
        raw = run_real_trajectory_writing_ik(jsonPath);
    end

    resampled = resample_task_trajectory_arclength( ...
        raw.task_trajectory, resamplingConfig);
    adapter = resampled_task_to_preik_segments(resampled);
    robot = build_legacy_robot();
    ikConfig = default_ik_config();
    postureConfig = default_writing_posture_config();
    candidateConfig = raw.candidate_config;

    % This is a fresh solve at resampled XYZ, not a lookup of camera-frame q.
    positionResultSet = solve_ik_segments( ...
        robot, adapter, raw.position_only_anchor, ikConfig);
    positionResultSet = attach_resampled_provenance_to_ik_result( ...
        positionResultSet, adapter);
    positionSummary = summarize_ik_continuity(positionResultSet);
    positionSeedLookup = build_position_only_seed_lookup(positionResultSet);
    writingResultSet = solve_writing_ik_segments( ...
        robot, adapter, raw.center_writing_seed, ...
        raw.candidate_task_plane_config, postureConfig, candidateConfig, ...
        positionSeedLookup, ...
        default_writing_recovery_policy('position_only_recovery'));
    writingResultSet = attach_resampled_provenance_to_ik_result( ...
        writingResultSet, adapter);
    writingSummary = writingResultSet.summary;

    rawCoverage = summarize_writing_arclength_coverage( ...
        raw.preik_segment_set, raw.writing_result_set);
    resampledCoverage = summarize_writing_arclength_coverage( ...
        adapter, writingResultSet);
    run = struct();
    run.raw_frame_driven_baseline = raw;
    run.resampling_config = resamplingConfig;
    run.resampled_task_trajectory = resampled;
    run.resampled_preik_adapter = adapter;
    run.resampled_position_only_result_set = positionResultSet;
    run.resampled_position_only_summary = positionSummary;
    run.resampled_position_only_seed_lookup = positionSeedLookup;
    run.resampled_writing_result_set = writingResultSet;
    run.resampled_writing_summary = writingSummary;
    run.raw_writing_coverage = rawCoverage;
    run.resampled_writing_coverage = resampledCoverage;
    run.ab_comparison = summarize_task_path_ik_ab( ...
        raw.preik_segment_set.summary.eligible_sample_count, raw.writing_summary, ...
        rawCoverage, adapter.summary.resampled_point_count, writingSummary, ...
        resampledCoverage);
end
