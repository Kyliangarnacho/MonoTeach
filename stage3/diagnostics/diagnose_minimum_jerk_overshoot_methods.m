function diagnostic = diagnose_minimum_jerk_overshoot_methods(robot, ...
        patchedMinimumJerk, writingResultSet, taskPlaneConfig, ...
        candidateConfig, postureConfig)
%DIAGNOSE_MINIMUM_JERK_OVERSHOOT_METHODS Fixed-segment overshoot comparison.
%
% Produces only the requested three final diagnostic methods: patched minimum
% jerk, plain cubic spline, and plain quintic Hermite.  The separate plain
% minimum-jerk artifact exists solely to report guard before/after evidence.

    validate_inputs(patchedMinimumJerk);
    inputSnapshot = patchedMinimumJerk;
    plainMinimumJerk = unpatched_minimum_jerk_artifact(patchedMinimumJerk);
    baselines = generate_polynomial_waypoint_diagnostic_baselines( ...
        patchedMinimumJerk, patchedMinimumJerk.timing_config);
    plainValidation = validate_continuous_writing_task_geometry(robot, ...
        plainMinimumJerk, writingResultSet, taskPlaneConfig, candidateConfig, ...
        postureConfig);
    patchedValidation = validate_continuous_writing_task_geometry(robot, ...
        patchedMinimumJerk, writingResultSet, taskPlaneConfig, candidateConfig, ...
        postureConfig);
    cubicValidation = validate_continuous_writing_task_geometry(robot, ...
        baselines.cubic, writingResultSet, taskPlaneConfig, candidateConfig, ...
        postureConfig);
    quinticValidation = validate_continuous_writing_task_geometry(robot, ...
        baselines.quintic, writingResultSet, taskPlaneConfig, candidateConfig, ...
        postureConfig);
    % Interior derivative boundary conditions intentionally use NaN to mean
    % "optimizer-selected".  Use isequaln so those sentinel values do not
    % look like a mutation of an otherwise value-semantic input artifact.
    if ~isequaln(patchedMinimumJerk, inputSnapshot)
        error('MonoTeach:ContinuousDiagnosticInputMutation', ...
            'Diagnostic must not modify its patched minimum-jerk input artifact.');
    end

    beforeMetric = metric_record('plain_minimum_jerk_before_patch', ...
        plainMinimumJerk, plainValidation, overshoot_before(patchedMinimumJerk));
    patchedMetric = metric_record('patched_minimum_jerk', patchedMinimumJerk, ...
        patchedValidation, overshoot_after(patchedMinimumJerk));
    cubicMetric = metric_record('plain_cubic_spline', baselines.cubic, ...
        cubicValidation, overshoot_from_baseline(baselines.cubic));
    quinticMetric = metric_record('plain_quintic_hermite', baselines.quintic, ...
        quinticValidation, overshoot_from_baseline(baselines.quintic));
    diagnostic = struct();
    diagnostic.artifact_type = 'ContinuousWritingTrajectoryDiagnostic';
    diagnostic.reference_time_policy = ...
        'patched_minimum_jerk_final_waypoint_times_shared_without_reallocation';
    diagnostic.plain_minimum_jerk_before_patch = struct( ...
        'trajectory', plainMinimumJerk, 'validation', plainValidation, ...
        'metrics', beforeMetric);
    diagnostic.patched_minimum_jerk = struct( ...
        'trajectory', patchedMinimumJerk, 'validation', patchedValidation, ...
        'metrics', patchedMetric);
    diagnostic.cubic = struct('trajectory', baselines.cubic, ...
        'validation', cubicValidation, 'metrics', cubicMetric);
    diagnostic.quintic = struct('trajectory', baselines.quintic, ...
        'validation', quinticValidation, 'metrics', quinticMetric);
    diagnostic.comparison_records = [patchedMetric, cubicMetric, quinticMetric];
end


function trajectory = unpatched_minimum_jerk_artifact(patched)

    trajectory = patched;
    trajectory.generation_method = 'plain_minimum_jerk_before_overshoot_patch';
    for index = 1:numel(trajectory.segments)
        segment = trajectory.segments(index);
        segment.q_rad = segment.unpatched_q_rad;
        segment.qd_rad_s = segment.unpatched_qd_rad_s;
        segment.qdd_rad_s2 = segment.unpatched_qdd_rad_s2;
        segment.qddd_rad_s3 = segment.unpatched_qddd_rad_s3;
        segment.piecewise_polynomial = segment.unpatched_piecewise_polynomial;
        trajectory.segments(index) = segment;
    end
end


function evidence = overshoot_before(trajectory)

    evidence = [trajectory.segments.overshoot_before_patch];
end


function evidence = overshoot_after(trajectory)

    evidence = [trajectory.segments.overshoot_after_patch];
end


function evidence = overshoot_from_baseline(trajectory)

    evidence = [trajectory.segments.overshoot_evidence];
end


function record = metric_record(method, trajectory, validation, evidences)

    segmentCount = numel(trajectory.segments);
    peakVelocity = zeros(segmentCount, 5);
    peakAcceleration = zeros(segmentCount, 5);
    duration = 0.0;
    for index = 1:segmentCount
        segment = trajectory.segments(index);
        peakVelocity(index, :) = max(abs(segment.qd_rad_s), [], 1);
        peakAcceleration(index, :) = max(abs(segment.qdd_rad_s2), [], 1);
        duration = duration + (segment.t_s(end) - segment.t_s(1));
    end
    record = struct();
    record.method = method;
    record.waypoint_time_points_s = {trajectory.segments.waypoint_t_s};
    record.max_joint_overshoot_rad = max([evidences.max_overshoot_magnitude_rad]);
    record.overshoot_record_count = sum([evidences.count]);
    record.mean_cartesian_deviation_m = ...
        validation.summary.mean_cartesian_path_deviation_m;
    record.max_cartesian_deviation_m = ...
        validation.summary.max_cartesian_path_deviation_m;
    record.max_tool_direction_error_deg = ...
        validation.summary.max_tool_direction_error_deg;
    record.minimum_joint_limit_margin = ...
        validation.summary.minimum_joint_limit_margin;
    record.peak_velocity_rad_s = max(peakVelocity, [], 1);
    record.peak_acceleration_rad_s2 = max(peakAcceleration, [], 1);
    record.duration_s = duration;
end


function validate_inputs(trajectory)

    required = {'artifact_type', 'timing_config', 'segments'};
    if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
            ~all(isfield(trajectory, required)) || ...
            ~strcmp(string(trajectory.artifact_type), "ContinuousJointTrajectory") || ...
            ~isstruct(trajectory.segments) || isempty(trajectory.segments)
        error('MonoTeach:InvalidMinimumJerkDiagnosticInput', ...
            'Diagnostic requires a nonempty patched ContinuousJointTrajectory.');
    end
    for index = 1:numel(trajectory.segments)
        segment = trajectory.segments(index);
        if ~isfield(segment, 'overshoot_before_patch') || ...
                ~isfield(segment, 'overshoot_after_patch') || ...
                ~isfield(segment, 'unpatched_q_rad') || ...
                ~isfield(segment, 'unpatched_qd_rad_s') || ...
                ~isfield(segment, 'unpatched_qdd_rad_s2') || ...
                ~isfield(segment, 'unpatched_qddd_rad_s3')
            error('MonoTeach:MissingMinimumJerkOvershootEvidence', ...
                'Patched trajectory must retain plain minimum-jerk evidence.');
        end
    end
end
