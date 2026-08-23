function execution = derive_recovered_writing_execution( ...
        preIkSegmentSet, strictWritingResultSet, recovery)
%DERIVE_RECOVERED_WRITING_EXECUTION Build post-recovery execution evidence.
%
% Strict successes are retained verbatim as source evidence.  Accepted
% relaxed points are inserted only in this derived artifact; unresolved
% points remain hard barriers, so no segment crosses an original semantic
% boundary or an unrecovered execution gap.

    validate_inputs(preIkSegmentSet, strictWritingResultSet, recovery);
    strictPoints = flatten_strict_points(strictWritingResultSet.segments);
    strictFailures = strictWritingResultSet.failures;
    recoveredPoints = flatten_recovered_points(recovery.window_results);
    segments = repmat(empty_segment(), 1, 0);
    remainingFailures = repmat(empty_failure(), 1, 0);

    for preIkIndex = 1:numel(preIkSegmentSet.segments)
        preIk = preIkSegmentSet.segments(preIkIndex);
        current = repmat(empty_execution_point(), 1, 0);
        for sampleIndex = 1:numel(preIk.source_indices)
            sourceIndex = preIk.source_indices(sampleIndex);
            strictMatch = find([strictPoints.source_index] == sourceIndex, 1, 'first');
            if isempty(recoveredPoints)
                recoveredMatch = [];
            else
                recoveredMatch = find([recoveredPoints.source_index] == sourceIndex, ...
                    1, 'first');
            end
            if ~isempty(strictMatch)
                current(end + 1) = execution_point_from_strict( ...
                    strictPoints(strictMatch)); %#ok<AGROW>
                continue;
            end
            if ~isempty(recoveredMatch)
                current(end + 1) = execution_point_from_recovery( ...
                    recoveredPoints(recoveredMatch)); %#ok<AGROW>
                continue;
            end
            failureMatch = find([strictFailures.source_index] == sourceIndex, 1, 'first');
            if isempty(failureMatch)
                error('MonoTeach:IncompleteRecoveredWritingEvidence', ...
                    'Every non-executed Pre-IK sample must retain strict failure evidence.');
            end
            if ~isempty(current)
                segments(end + 1) = make_segment(preIkIndex, current); %#ok<AGROW>
            end
            current = repmat(empty_execution_point(), 1, 0);
            remainingFailures(end + 1) = failure_from_strict( ...
                strictFailures(failureMatch)); %#ok<AGROW>
        end
        if ~isempty(current)
            segments(end + 1) = make_segment(preIkIndex, current); %#ok<AGROW>
        end
    end

    execution = struct();
    execution.artifact_type = 'RecoveredWritingExecution';
    execution.strict_writing_result_set = strictWritingResultSet;
    execution.recovery_artifact = recovery;
    execution.segments = segments;
    execution.remaining_failures = remainingFailures;
    execution.summary = struct( ...
        'writing_success_point_count_before', strictWritingResultSet.summary.success_point_count, ...
        'writing_execution_segment_count_before', ...
            strictWritingResultSet.summary.ik_success_segment_count, ...
        'recovered_point_count', numel(recoveredPoints), ...
        'remaining_failure_count', numel(remainingFailures), ...
        'writing_success_point_count_after', ...
            strictWritingResultSet.summary.success_point_count + numel(recoveredPoints), ...
        'writing_execution_segment_count_after', numel(segments), ...
        'fully_recovered_gap_count', recovery.summary.fully_reconnected_gap_count);
end


function points = flatten_strict_points(segments)

    points = struct([]);
    for index = 1:numel(segments)
        if isempty(points), points = segments(index).results;
        else, points = [points, segments(index).results]; end %#ok<AGROW>
    end
end


function points = flatten_recovered_points(windowResults)

    points = struct([]);
    for index = 1:numel(windowResults)
        recovered = windowResults(index).recovered_points;
        if isempty(recovered), continue; end
        if isempty(points), points = recovered;
        else, points = [points, recovered]; end %#ok<AGROW>
    end
end


function point = execution_point_from_strict(strict)

    point = empty_execution_point();
    point.source_index = strict.source_index;
    point.t_ms = strict.t_ms;
    point.target_xyz_m = strict.target_xyz_m;
    point.q = strict.q;
    point.fk_xyz_m = strict.fk_xyz_m;
    point.position_error_m = strict.position_error_m;
    point.tool_direction_error_deg = strict.tool_direction_error_deg;
    point.joint_limit_margin = strict.joint_limit_margin;
    point.execution_kind = 'strict_writing';
    point.provenance = provenance_from_record(strict);
end


function point = execution_point_from_recovery(recovered)

    point = empty_execution_point();
    point.source_index = recovered.source_index;
    point.t_ms = recovered.t_ms;
    point.target_xyz_m = recovered.original_target_xyz_m;
    point.q = recovered.q;
    point.fk_xyz_m = recovered.fk_xyz_m;
    point.position_error_m = recovered.position_error_m;
    point.tool_direction_error_deg = recovered.tool_direction_error_deg;
    point.joint_limit_margin = recovered.joint_limit_margin;
    point.execution_kind = 'orientation_relaxed_recovery';
    point.provenance = recovered.provenance;
end


function failure = failure_from_strict(strict)

    failure = empty_failure();
    failure.source_index = strict.source_index;
    failure.t_ms = strict.t_ms;
    failure.target_xyz_m = strict.target_xyz_m;
    failure.reason = strict.reason;
    failure.provenance = provenance_from_record(strict);
end


function provenance = provenance_from_record(record)

    if isfield(record, 'resampled_provenance')
        provenance = record.resampled_provenance;
    elseif isfield(record, 'provenance')
        provenance = record.provenance;
    else
        provenance = struct('source_index', record.source_index, ...
            'kind', 'source_index_only');
    end
end


function segment = make_segment(preIkIndex, points)

    segment = empty_segment();
    segment.preik_segment_index = preIkIndex;
    segment.source_indices = [points.source_index];
    segment.q_rad = vertcat(points.q);
    segment.points = points;
end


function value = empty_segment()

    value = struct('preik_segment_index', NaN, 'source_indices', zeros(1, 0), ...
        'q_rad', zeros(0, 0), 'points', struct([]));
end


function value = empty_execution_point()

    value = struct('source_index', NaN, 't_ms', NaN, ...
        'target_xyz_m', NaN(1, 3), 'q', [], 'fk_xyz_m', NaN(1, 3), ...
        'position_error_m', NaN, 'tool_direction_error_deg', NaN, ...
        'joint_limit_margin', [], 'execution_kind', '', 'provenance', struct([]));
end


function value = empty_failure()

    value = struct('source_index', NaN, 't_ms', NaN, ...
        'target_xyz_m', NaN(1, 3), 'reason', '', 'provenance', struct([]));
end


function validate_inputs(preIk, strict, recovery)

    if ~isstruct(preIk) || ~isfield(preIk, 'segments') || ...
            ~isstruct(strict) || ~isfield(strict, 'segments') || ...
            ~isfield(strict, 'failures') || ~isfield(strict, 'summary') || ...
            ~isstruct(recovery) || ~isfield(recovery, 'window_results') || ...
            ~isfield(recovery, 'summary')
        error('MonoTeach:InvalidRecoveredWritingExecutionInput', ...
            'Pre-IK, strict Writing and recovery artifacts are required.');
    end
end
