function plan = build_backward_writing_rescue_plan(segmentSet, forwardResultSet)
%BUILD_BACKWARD_WRITING_RESCUE_PLAN Locate next forward success per failure.
%
% Planning is performed independently for each Pre-IK segment, so an invalid
% Stage 3.1 barrier can never supply a seed across that boundary.

    if ~isstruct(segmentSet) || ~isfield(segmentSet, 'segments') || ...
            ~isstruct(forwardResultSet) || ~isfield(forwardResultSet, 'segments') || ...
            ~isfield(forwardResultSet, 'failures')
        error('MonoTeach:InvalidBackwardWritingRescuePlanInput', ...
            'Segment set and forward Writing result set are required.');
    end
    successes = flatten_successes(forwardResultSet.segments);
    failures = forwardResultSet.failures;
    records = repmat(empty_record(), 1, 0);
    for segmentIndex = 1:numel(segmentSet.segments)
        sourceIndices = segmentSet.segments(segmentIndex).source_indices;
        nextQ = [];
        nextSourceIndex = NaN;
        for index = numel(sourceIndices):-1:1
            sourceIndex = sourceIndices(index);
            successIndex = find([successes.source_index] == sourceIndex, 1, 'first');
            if ~isempty(successIndex)
                nextQ = successes(successIndex).q;
                nextSourceIndex = sourceIndex;
                continue;
            end
            failureIndex = find([failures.source_index] == sourceIndex, 1, 'first');
            if isempty(failureIndex)
                error('MonoTeach:IncompleteForwardWritingResultSet', ...
                    'Every Pre-IK source must be a forward success or failure.');
            end
            record = empty_record();
            record.preik_segment_index = segmentIndex;
            record.source_index = sourceIndex;
            record.forward_failure_index = failureIndex;
            record.next_success_source_index = nextSourceIndex;
            record.next_success_q = nextQ;
            record.eligible_for_backward_rescue = ~isempty(nextQ);
            records(end + 1) = record; %#ok<AGROW>
        end
    end
    plan = struct('records', records, ...
        'forward_failure_source_indices', [failures.source_index]);
end


function successes = flatten_successes(segments)

    successes = struct([]);
    for segmentIndex = 1:numel(segments)
        if isempty(successes)
            successes = segments(segmentIndex).results;
        else
            successes = [successes, segments(segmentIndex).results]; %#ok<AGROW>
        end
    end
end


function record = empty_record()

    record = struct('preik_segment_index', NaN, 'source_index', NaN, ...
        'forward_failure_index', NaN, 'next_success_source_index', NaN, ...
        'next_success_q', [], 'eligible_for_backward_rescue', false);
end
