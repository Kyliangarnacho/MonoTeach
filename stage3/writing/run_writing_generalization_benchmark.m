function benchmark = run_writing_generalization_benchmark(fixtures, config)
%RUN_WRITING_GENERALIZATION_BENCHMARK Fixed-config Stage 3.3 path gate.
%
% Each fixture runs independently through mapping, 2 mm resampling, fresh
% Position-Only IK, and the existing forward Writing recovery ladder.

    if nargin < 1 || isempty(fixtures)
        fixtures = make_writing_generalization_fixtures();
    end
    if nargin < 2 || isempty(config)
        config = default_writing_generalization_benchmark_config();
    end
    validate_frozen_config(config);
    validate_fixtures(fixtures, config);
    configBefore = config;
    baselinePlane = default_task_plane_config();
    candidatePlane = build_writing_candidate_task_plane_config( ...
        baselinePlane, config.candidate_config);
    robot = build_legacy_robot();
    positionAnchor = solve_anchor_ik(robot, candidatePlane, config.ik_config);
    centerSeed = resolve_writing_candidate_center_seed(robot, candidatePlane, ...
        config.ik_config, config.writing_posture_config, config.candidate_config);
    records = repmat(empty_record(), 1, 0);
    for fixtureIndex = 1:numel(fixtures)
        fixture = fixtures(fixtureIndex);
        task = workspace_to_task_trajectory( ...
            fixture.workspace_trajectory, candidatePlane);
        resampled = resample_task_trajectory_arclength( ...
            task, config.resampling_config);
        adapter = resampled_task_to_preik_segments(resampled);
        positionResult = solve_ik_segments(robot, adapter, positionAnchor, ...
            config.ik_config);
        positionResult = attach_resampled_provenance_to_ik_result( ...
            positionResult, adapter);
        positionSummary = summarize_ik_continuity(positionResult);
        positionLookup = build_position_only_seed_lookup(positionResult);
        writingResult = solve_writing_ik_segments(robot, adapter, centerSeed, ...
            candidatePlane, config.writing_posture_config, ...
            config.candidate_config, positionLookup, config.recovery_policy);
        writingResult = attach_resampled_provenance_to_ik_result( ...
            writingResult, adapter);
        coverage = summarize_writing_arclength_coverage(adapter, writingResult);
        records(end + 1) = build_record(fixture, resampled, positionSummary, ...
            writingResult.summary, coverage, config); %#ok<AGROW>
    end
    if ~isequal(config, configBefore)
        error('MonoTeach:BenchmarkConfigMutationDetected', ...
            'Benchmark fixtures must not modify frozen configuration.');
    end
    benchmark = struct();
    benchmark.artifact_type = 'writing_generalization_benchmark';
    benchmark.frozen_config = config;
    benchmark.records = sort_records_by_name(records);
    benchmark.summary_table = benchmark.records;
end


function record = build_record(fixture, resampled, positionSummary, writingSummary, coverage, config)
    record = struct();
    record.trajectory_name = fixture.name;
    record.fixture_kind = fixture.fixture_kind;
    record.path_length_m = coverage.total_arclength_m;
    record.resampled_point_count = resampled.summary.resampled_point_count;
    record.position_only_success_point_count = positionSummary.success_point_count;
    record.position_only_failure_count = positionSummary.failure_count;
    record.position_only_success_rate = positionSummary.ik_success_rate;
    record.writing_success_point_count = writingSummary.success_point_count;
    record.writing_failure_count = writingSummary.failure_count;
    record.writing_success_rate = writingSummary.ik_success_rate;
    record.accepted_sampled_arclength_m = coverage.accepted_sampled_arclength_m;
    record.accepted_sampled_coverage_fraction = ...
        coverage.accepted_sampled_coverage_fraction;
    record.ik_success_segment_count = writingSummary.ik_success_segment_count;
    record.max_joint_step_rad = writingSummary.overall_max_joint_step;
    record.minimum_joint_limit_margin = writingSummary.minimum_joint_limit_margin;
    record.mean_position_error_m = writingSummary.mean_fk_position_error_m;
    record.max_position_error_m = writingSummary.max_fk_position_error_m;
    record.mean_direction_error_deg = writingSummary.mean_tool_direction_error_deg;
    record.max_direction_error_deg = writingSummary.max_tool_direction_error_deg;
    record.failure_arc_length_locations = coverage.failed_points;
    record.frozen_config = config;
end


function records = sort_records_by_name(records)
    [~, order] = sort({records.trajectory_name});
    records = records(order);
end


function validate_frozen_config(config)
    expected = default_writing_generalization_benchmark_config();
    if ~isequal(config, expected)
        error('MonoTeach:GeneralizationBenchmarkConfigNotFrozen', ...
            'Generalization benchmark config must equal the frozen defaults.');
    end
end


function validate_fixtures(fixtures, config)
    if ~isstruct(fixtures) || isempty(fixtures)
        error('MonoTeach:InvalidGeneralizationFixtures', ...
            'At least one WorkspaceTrajectory fixture is required.');
    end
    for index = 1:numel(fixtures)
        fixture = fixtures(index);
        if ~isfield(fixture, 'name') || ~isfield(fixture, 'workspace_trajectory') || ...
                ~isfield(fixture, 'fixture_kind')
            error('MonoTeach:InvalidGeneralizationFixture', ...
                'Fixture records require name, kind, and workspace trajectory.');
        end
        validate_workspace_trajectory(fixture.workspace_trajectory);
        metadata = fixture.workspace_trajectory.metadata;
        if metadata.width_mm ~= config.workspace_width_mm || ...
                metadata.height_mm ~= config.workspace_height_mm
            error('MonoTeach:GeneralizationWorkspaceContractMismatch', ...
                'Every fixture must use the frozen 190-by-290 mm Workspace contract.');
        end
    end
end


function record = empty_record()
    record = struct('trajectory_name', '', 'fixture_kind', '', ...
        'path_length_m', NaN, 'resampled_point_count', NaN, ...
        'position_only_success_point_count', NaN, ...
        'position_only_failure_count', NaN, ...
        'position_only_success_rate', NaN, ...
        'writing_success_point_count', NaN, ...
        'writing_failure_count', NaN, 'writing_success_rate', NaN, ...
        'accepted_sampled_arclength_m', NaN, ...
        'accepted_sampled_coverage_fraction', NaN, ...
        'ik_success_segment_count', NaN, 'max_joint_step_rad', NaN, ...
        'minimum_joint_limit_margin', NaN, 'mean_position_error_m', NaN, ...
        'max_position_error_m', NaN, 'mean_direction_error_deg', NaN, ...
        'max_direction_error_deg', NaN, ...
        'failure_arc_length_locations', struct([]), 'frozen_config', struct());
end
