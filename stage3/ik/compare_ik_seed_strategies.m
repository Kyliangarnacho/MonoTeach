function comparison = compare_ik_seed_strategies( ...
        robot, ...
        segmentSet, ...
        qAnchorOrResult, ...
        ikConfig)
%COMPARE_IK_SEED_STRATEGIES Compare anchor-only and previous-success seeds.
%
% Strategy A solves every candidate sample from q_anchor. Strategy B is the
% formal solve_ik_segments policy, which seeds successive accepted points from
% previous q and resets to q_anchor across pre-IK or failure barriers.
%
% This analysis does not change solve_ik_segments. For Strategy A, reported
% joint steps are analysis-only differences between adjacent recovered q values
% in the same successful source run; they are not written into canonical
% delta_q fields and are not velocities. The comparison is diagnostic: it can
% expose seed-sensitive branch jumps, not certify execution safety.

    previousSuccessSet = solve_ik_segments( ...
        robot, ...
        segmentSet, ...
        qAnchorOrResult, ...
        ikConfig);
    previousSummary = summarize_ik_continuity(previousSuccessSet);

    qAnchor = resolve_anchor_q(qAnchorOrResult);
    anchorOnly = solve_anchor_seed_strategy( ...
        robot, ...
        segmentSet, ...
        qAnchor, ...
        ikConfig);

    comparison = struct();
    comparison.anchor_seed = anchorOnly.summary;
    comparison.previous_success_seed = struct( ...
        'success_point_count', previousSummary.success_point_count, ...
        'failure_count', previousSummary.failure_count, ...
        'ik_success_rate', previousSummary.ik_success_rate, ...
        'mean_abs_joint_step_rad', previousSummary.mean_abs_delta_q, ...
        'max_abs_joint_step_rad', previousSummary.overall_max_joint_step, ...
        'largest_jump_source_index', previousSummary.largest_jump_source_index, ...
        'largest_jump_joint_index', previousSummary.largest_jump_joint_index);
    comparison.max_abs_joint_step_difference_rad = ...
        anchorOnly.summary.max_abs_joint_step_rad - ...
        previousSummary.overall_max_joint_step;
end


function strategy = solve_anchor_seed_strategy( ...
        robot, ...
        segmentSet, ...
        qAnchor, ...
        ikConfig)

    successfulResults = struct([]);
    failures = struct([]);
    interpointSteps = zeros(0, 5);
    stepSourceIndices = zeros(1, 0);

    for preIkSegmentIndex = 1:numel(segmentSet.segments)
        preIkSegment = segmentSet.segments(preIkSegmentIndex);
        previousQ = [];

        for sampleIndex = 1:numel(preIkSegment.samples)
            oneSampleSet = single_sample_segment_set( ...
                preIkSegment, ...
                sampleIndex);
            oneSampleResultSet = solve_ik_segments( ...
                robot, ...
                oneSampleSet, ...
                qAnchor, ...
                ikConfig);

            if oneSampleResultSet.summary.success_point_count == 1
                result = oneSampleResultSet.segments(1).results(1);
                successfulResults = append_result(successfulResults, result);

                if ~isempty(previousQ)
                    interpointSteps(end + 1, :) = result.q - previousQ; %#ok<AGROW>
                    stepSourceIndices(end + 1) = result.source_index; %#ok<AGROW>
                end

                previousQ = result.q;
            else
                failures = append_failures(failures, ...
                    oneSampleResultSet.failures, ...
                    preIkSegmentIndex);
                previousQ = [];
            end
        end
    end

    strategy = struct();
    strategy.results = successfulResults;
    strategy.failures = failures;
    strategy.summary = anchor_seed_summary( ...
        successfulResults, ...
        failures, ...
        interpointSteps, ...
        stepSourceIndices);
end


function segmentSet = single_sample_segment_set(preIkSegment, sampleIndex)

    segment = preIkSegment;
    segment.source_indices = preIkSegment.source_indices(sampleIndex);
    segment.samples = preIkSegment.samples(sampleIndex);

    segmentSet = struct( ...
        'coordinate_frame', 'robot_base', ...
        'units', 'm', ...
        'segments', segment);
end


function summary = anchor_seed_summary(results, failures, steps, stepSourceIndices)

    successCount = numel(results);
    failureCount = numel(failures);
    attemptCount = successCount + failureCount;

    summary = struct();
    summary.success_point_count = successCount;
    summary.failure_count = failureCount;

    if attemptCount == 0
        summary.ik_success_rate = NaN;
    else
        summary.ik_success_rate = successCount / attemptCount;
    end

    if isempty(steps)
        summary.mean_abs_joint_step_rad = NaN;
        summary.max_abs_joint_step_rad = NaN;
        summary.largest_jump_source_index = [];
        summary.largest_jump_joint_index = [];
        return;
    end

    absoluteSteps = abs(steps);
    summary.mean_abs_joint_step_rad = mean(absoluteSteps, 'all');
    [summary.max_abs_joint_step_rad, flatIndex] = max(absoluteSteps, [], 'all');
    [stepIndex, jointIndex] = ind2sub(size(absoluteSteps), flatIndex);
    summary.largest_jump_source_index = stepSourceIndices(stepIndex);
    summary.largest_jump_joint_index = jointIndex;
end


function qAnchor = resolve_anchor_q(qAnchorOrResult)

    if isnumeric(qAnchorOrResult)
        qAnchor = qAnchorOrResult;
    elseif isstruct(qAnchorOrResult) && isscalar(qAnchorOrResult) && ...
            isfield(qAnchorOrResult, 'q')
        qAnchor = qAnchorOrResult.q;
    else
        error( ...
            'MonoTeach:InvalidSeedStrategyAnchor', ...
            'qAnchorOrResult must be one q row vector or result with .q.');
    end

    if ~isnumeric(qAnchor) || ~isreal(qAnchor) || ...
            ~isequal(size(qAnchor), [1, 5]) || any(~isfinite(qAnchor))
        error( ...
            'MonoTeach:InvalidSeedStrategyAnchor', ...
            'q_anchor must be one finite 1-by-5 row vector.');
    end
end


function results = append_result(results, result)

    if isempty(results)
        results = result;
    else
        results(end + 1) = result; %#ok<AGROW>
    end
end


function failures = append_failures(failures, newFailures, preIkSegmentIndex)

    for i = 1:numel(newFailures)
        failure = newFailures(i);
        failure.preik_segment_index = preIkSegmentIndex;

        if isempty(failures)
            failures = failure;
        else
            failures(end + 1) = failure; %#ok<AGROW>
        end
    end
end
