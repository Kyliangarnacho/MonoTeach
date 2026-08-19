function continuity = summarize_ik_continuity(ikResultSet)
%SUMMARIZE_IK_CONTINUITY Derive read-only continuity diagnostics from IKResultSet.
%
% delta_q is a joint-position step between adjacent successful points in one
% IK-success subsegment. It is not a velocity: no sample interval, resampling,
% or time derivative is applied here. Large steps can reveal a branch change,
% but do not alone establish safety because speed, acceleration, collision,
% and execution limits are outside this stage.
%
% source_index is retained so the largest step and closest limit can be traced
% back to the originating Stage 3.1 sample. For a step, its source is the
% endpoint/current point whose delta_q was calculated.

    validate_ik_result_set(ikResultSet);

    pointResults = collect_point_results(ikResultSet.segments);
    successPointCount = numel(pointResults);
    failureCount = numel(ikResultSet.failures);
    attemptCount = successPointCount + failureCount;

    continuity = struct();
    continuity.success_point_count = successPointCount;
    continuity.failure_count = failureCount;

    if attemptCount == 0
        continuity.ik_success_rate = NaN;
    else
        continuity.ik_success_rate = successPointCount / attemptCount;
    end

    if isempty(pointResults)
        continuity.mean_fk_position_error_m = NaN;
        continuity.max_fk_position_error_m = NaN;
        continuity.per_joint_max_abs_delta_q = NaN(1, 5);
        continuity.mean_abs_delta_q = NaN;
        continuity.overall_max_joint_step = NaN;
        continuity.largest_jump_source_index = [];
        continuity.largest_jump_joint_index = [];
        continuity.minimum_joint_limit_margin = NaN;
        continuity.closest_limit_joint_index = [];
        continuity.closest_limit_source_index = [];
        return;
    end

    fkErrors = [pointResults.position_error_m];
    continuity.mean_fk_position_error_m = mean(fkErrors);
    continuity.max_fk_position_error_m = max(fkErrors);

    [deltaQ, deltaSourceIndices] = collect_delta_q(pointResults);

    if isempty(deltaQ)
        continuity.per_joint_max_abs_delta_q = NaN(1, 5);
        continuity.mean_abs_delta_q = NaN;
        continuity.overall_max_joint_step = NaN;
        continuity.largest_jump_source_index = [];
        continuity.largest_jump_joint_index = [];
    else
        absDeltaQ = abs(deltaQ);
        continuity.per_joint_max_abs_delta_q = max(absDeltaQ, [], 1);
        continuity.mean_abs_delta_q = mean(absDeltaQ, 'all');
        [continuity.overall_max_joint_step, flatIndex] = max(absDeltaQ, [], 'all');
        [stepIndex, jointIndex] = ind2sub(size(absDeltaQ), flatIndex);
        continuity.largest_jump_source_index = deltaSourceIndices(stepIndex);
        continuity.largest_jump_joint_index = jointIndex;
    end

    [marginMatrix, marginSourceIndices] = collect_joint_margins(pointResults);
    [continuity.minimum_joint_limit_margin, flatIndex] = min(marginMatrix, [], 'all');
    [pointIndex, jointIndex] = ind2sub(size(marginMatrix), flatIndex);
    continuity.closest_limit_joint_index = jointIndex;
    continuity.closest_limit_source_index = marginSourceIndices(pointIndex);
end


function pointResults = collect_point_results(successSegments)

    pointResults = struct([]);

    for i = 1:numel(successSegments)
        results = successSegments(i).results;

        if isempty(results)
            continue;
        end

        if isempty(pointResults)
            pointResults = results;
        else
            pointResults = [pointResults, results]; %#ok<AGROW>
        end
    end
end


function [deltaQ, sourceIndices] = collect_delta_q(pointResults)

    deltaQ = zeros(0, 5);
    sourceIndices = zeros(1, 0);

    for i = 1:numel(pointResults)
        delta = pointResults(i).delta_q;

        if isempty(delta)
            continue;
        end

        deltaQ(end + 1, :) = delta; %#ok<AGROW>
        sourceIndices(end + 1) = pointResults(i).source_index; %#ok<AGROW>
    end
end


function [marginMatrix, sourceIndices] = collect_joint_margins(pointResults)

    marginMatrix = vertcat(pointResults.joint_limit_margin);
    sourceIndices = [pointResults.source_index];
end


function validate_ik_result_set(ikResultSet)

    if ~isstruct(ikResultSet) || ~isscalar(ikResultSet) || ...
            ~isfield(ikResultSet, 'coordinate_space') || ...
            ~isfield(ikResultSet, 'units') || ...
            ~isfield(ikResultSet, 'segments') || ...
            ~isfield(ikResultSet, 'failures') || ...
            ~strcmp(string(ikResultSet.coordinate_space), "joint") || ...
            ~strcmp(string(ikResultSet.units), "rad") || ...
            ~isstruct(ikResultSet.segments) || ...
            ~isstruct(ikResultSet.failures)
        error( ...
            'MonoTeach:InvalidIKResultSet', ...
            'ikResultSet must be one joint/rad result set with segments and failures.');
    end

    for i = 1:numel(ikResultSet.segments)
        segment = ikResultSet.segments(i);

        if ~isfield(segment, 'results') || ~isstruct(segment.results)
            error( ...
                'MonoTeach:InvalidIKSuccessSegment', ...
                'Each IK success segment must contain canonical results.');
        end

        for j = 1:numel(segment.results)
            result = segment.results(j);
            requiredFields = { ...
                'source_index', ...
                'position_error_m', ...
                'delta_q', ...
                'joint_limit_margin'};

            if ~all(isfield(result, requiredFields)) || ...
                    ~isscalar(result.position_error_m) || ...
                    ~isfinite(result.position_error_m) || ...
                    ~isequal(size(result.joint_limit_margin), [1, 5]) || ...
                    any(~isfinite(result.joint_limit_margin))
                error( ...
                    'MonoTeach:InvalidIKPointResult', ...
                    'Each canonical success result must contain finite diagnostics.');
            end

            if ~isempty(result.delta_q) && ...
                    (~isequal(size(result.delta_q), [1, 5]) || ...
                    any(~isfinite(result.delta_q)))
                error( ...
                    'MonoTeach:InvalidIKPointDelta', ...
                    'delta_q must be empty or one finite 1-by-5 joint step.');
            end
        end
    end
end
