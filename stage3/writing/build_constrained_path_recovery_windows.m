function windows = build_constrained_path_recovery_windows( ...
        preIkSegmentSet, strictWritingResultSet)
%BUILD_CONSTRAINED_PATH_RECOVERY_WINDOWS Identify recoverable internal gaps.
%
% A window exists only when a contiguous failed span is bracketed by strict
% Writing successes inside one Pre-IK segment.  The function never joins
% different Pre-IK segments, so invalid/outside barriers remain semantic
% barriers rather than recoverable execution gaps.

    validate_inputs(preIkSegmentSet, strictWritingResultSet);
    strictPoints = flatten_points(strictWritingResultSet.segments);
    strictFailures = strictWritingResultSet.failures;
    windows = repmat(empty_window(), 1, 0);

    for segmentIndex = 1:numel(preIkSegmentSet.segments)
        segment = preIkSegmentSet.segments(segmentIndex);
        states = cell(1, numel(segment.source_indices));
        for pointIndex = 1:numel(segment.source_indices)
            sourceIndex = segment.source_indices(pointIndex);
            successMatch = find([strictPoints.source_index] == sourceIndex, 1, 'first');
            failureMatch = find([strictFailures.source_index] == sourceIndex, 1, 'first');
            if ~isempty(successMatch) && isempty(failureMatch)
                states{pointIndex} = struct('kind', 'success', ...
                    'record', strictPoints(successMatch));
            elseif isempty(successMatch) && ~isempty(failureMatch)
                states{pointIndex} = struct('kind', 'failure', ...
                    'record', strictFailures(failureMatch));
            else
                error('MonoTeach:IncompleteStrictWritingEvidence', ...
                    ['Every eligible Pre-IK sample must have exactly one strict ' ...
                     'Writing success or failure record.']);
            end
        end

        index = 1;
        while index <= numel(states)
            if ~strcmp(states{index}.kind, 'failure')
                index = index + 1;
                continue;
            end
            firstFailure = index;
            while index <= numel(states) && strcmp(states{index}.kind, 'failure')
                index = index + 1;
            end
            lastFailure = index - 1;
            if firstFailure == 1 || index > numel(states) || ...
                    ~strcmp(states{firstFailure - 1}.kind, 'success') || ...
                    ~strcmp(states{index}.kind, 'success')
                continue;
            end
            windows(end + 1) = build_window(segmentIndex, segment, states, ...
                firstFailure - 1, firstFailure:lastFailure, index); %#ok<AGROW>
            windows(end).gap_index = numel(windows);
        end
    end
end


function window = build_window(segmentIndex, segment, states, leftIndex, ...
        failedIndices, rightIndex)

    left = states{leftIndex}.record;
    right = states{rightIndex}.record;
    failed = repmat(states{failedIndices(1)}.record, 1, numel(failedIndices));
    for failedIndex = 1:numel(failedIndices)
        failed(failedIndex) = states{failedIndices(failedIndex)}.record;
    end
    targets = repmat(empty_target(), 1, numel(failed));
    provenance = cell(1, numel(failed));
    for index = 1:numel(failed)
        targets(index) = target_from_failure(failed(index));
        provenance{index} = provenance_from_record(failed(index));
    end
    window = empty_window();
    window.gap_index = NaN;
    window.preik_segment_index = segmentIndex;
    window.original_run_index = field_or_default(segment, 'original_run_index', NaN);
    window.left_success_index = left.source_index;
    window.right_success_index = right.source_index;
    window.failed_indices = [failed.source_index];
    window.left_accepted_q = left.q;
    window.right_accepted_q = right.q;
    window.left_success_provenance = provenance_from_record(left);
    window.right_success_provenance = provenance_from_record(right);
    window.failed_source_provenance = provenance;
    window.original_writing_targets = targets;
    window.strict_failure_reasons = {failed.reason};
    window.strict_failure_records = failed;
end


function target = target_from_failure(failure)

    target = empty_target();
    target.source_index = failure.source_index;
    target.t_ms = failure.t_ms;
    target.target_xyz_m = failure.target_xyz_m;
    target.target_tool_direction = failure.signed_normal_base;
    target.aim_target_m = failure.aim_target_m;
    target.provenance = provenance_from_record(failure);
end


function provenance = provenance_from_record(record)

    if isfield(record, 'resampled_provenance')
        provenance = record.resampled_provenance;
    else
        provenance = struct('source_index', record.source_index, ...
            'kind', 'source_index_only');
    end
end


function value = field_or_default(input, fieldName, defaultValue)

    if isfield(input, fieldName), value = input.(fieldName);
    else, value = defaultValue; end
end


function points = flatten_points(segments)

    points = struct([]);
    for index = 1:numel(segments)
        if isempty(points), points = segments(index).results;
        else, points = [points, segments(index).results]; end %#ok<AGROW>
    end
end


function window = empty_window()

    window = struct('gap_index', NaN, 'preik_segment_index', NaN, ...
        'original_run_index', NaN, 'left_success_index', NaN, ...
        'right_success_index', NaN, 'failed_indices', zeros(1, 0), ...
        'left_accepted_q', [], 'right_accepted_q', [], ...
        'left_success_provenance', struct([]), ...
        'right_success_provenance', struct([]), ...
        'failed_source_provenance', {{}}, ...
        'original_writing_targets', struct([]), ...
        'strict_failure_reasons', {{}}, 'strict_failure_records', struct([]));
end


function target = empty_target()

    target = struct('source_index', NaN, 't_ms', NaN, ...
        'target_xyz_m', NaN(1, 3), 'target_tool_direction', NaN(1, 3), ...
        'aim_target_m', NaN(1, 3), 'provenance', struct([]));
end


function validate_inputs(preIk, strict)

    if ~isstruct(preIk) || ~isfield(preIk, 'segments') || ...
            ~isstruct(strict) || ~isfield(strict, 'segments') || ...
            ~isfield(strict, 'failures')
        error('MonoTeach:InvalidConstrainedGapWindowInput', ...
            'Pre-IK segments and a strict Writing result set are required.');
    end
end
