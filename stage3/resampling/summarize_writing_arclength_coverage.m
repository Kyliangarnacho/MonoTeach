function coverage = summarize_writing_arclength_coverage(segmentSet, writingResultSet)
%SUMMARIZE_WRITING_ARCLENGTH_COVERAGE Discrete path coverage without bridging.
%
% Accepted coverage is the sum of arc-length intervals whose endpoints both
% belong to one IK-success subsegment.  It is sampled-path evidence, not a
% claim that every continuous point between waypoints is reachable.

    arcs = segment_arclength_records(segmentSet);
    runIndices = unique([arcs.original_run_index]);
    totalLength = 0.0;
    for runIndex = runIndices
        totalLength = totalLength + max([arcs( ...
            [arcs.original_run_index] == runIndex).run_total_length_m]);
    end
    acceptedLength = 0.0;
    for segmentIndex = 1:numel(writingResultSet.segments)
        results = writingResultSet.segments(segmentIndex).results;
        if numel(results) < 2, continue; end
        values = zeros(1, numel(results));
        for resultIndex = 1:numel(results)
            record = find_record(arcs, results(resultIndex).source_index);
            values(resultIndex) = record.source_arclength_m;
        end
        acceptedLength = acceptedLength + sum(diff(values));
    end
    failures = writingResultSet.failures;
    failedPoints = repmat(empty_failed_point(), 1, 0);
    for failureIndex = 1:numel(failures)
        record = find_record(arcs, failures(failureIndex).source_index);
        failedPoints(end + 1) = struct( ...
            'source_index', failures(failureIndex).source_index, ...
            'original_run_index', record.original_run_index, ...
            'source_arclength_m', record.source_arclength_m, ...
            'source_bracket_indices', record.source_bracket_indices, ...
            'reason', failures(failureIndex).reason); %#ok<AGROW>
    end
    coverage = struct();
    coverage.total_arclength_m = totalLength;
    coverage.accepted_sampled_arclength_m = acceptedLength;
    if totalLength <= eps, coverage.accepted_sampled_coverage_fraction = NaN;
    else, coverage.accepted_sampled_coverage_fraction = acceptedLength / totalLength; end
    coverage.failed_points = failedPoints;
    coverage.failed_point_count = numel(failedPoints);
end


function records = segment_arclength_records(segmentSet)
    records = repmat(empty_record(), 1, 0);
    for segmentIndex = 1:numel(segmentSet.segments)
        segment = segmentSet.segments(segmentIndex);
        samples = segment.samples;
        if isfield(samples, 'source_arclength_m')
            arc = [samples.source_arclength_m];
        else
            xyz = [[samples.x_m]' [samples.y_m]' [samples.z_m]'];
            arc = [0.0, cumsum(vecnorm(diff(xyz, 1, 1), 2, 2)')];
        end
        for index = 1:numel(samples)
            if isfield(samples, 'source_bracket_indices')
                bracket = samples(index).source_bracket_indices;
                originalRunIndex = samples(index).original_run_index;
            else
                bracket = [segment.source_indices(index), segment.source_indices(index)];
                originalRunIndex = segmentIndex;
            end
            records(end + 1) = struct( ...
                'source_index', segment.source_indices(index), ...
                'original_run_index', originalRunIndex, ...
                'source_arclength_m', arc(index), ...
                'source_bracket_indices', bracket, ...
                'run_total_length_m', arc(end)); %#ok<AGROW>
        end
    end
end


function record = find_record(records, sourceIndex)
    index = find([records.source_index] == sourceIndex, 1, 'first');
    if isempty(index)
        error('MonoTeach:MissingWritingArclengthRecord', ...
            'No arc-length record for IK source_index %d.', sourceIndex);
    end
    record = records(index);
end


function point = empty_failed_point()
    point = struct('source_index', NaN, 'original_run_index', NaN, ...
        'source_arclength_m', NaN, 'source_bracket_indices', NaN(1,2), ...
        'reason', '');
end


function record = empty_record()
    record = struct('source_index', NaN, 'original_run_index', NaN, ...
        'source_arclength_m', NaN, 'source_bracket_indices', NaN(1,2), ...
        'run_total_length_m', NaN);
end
