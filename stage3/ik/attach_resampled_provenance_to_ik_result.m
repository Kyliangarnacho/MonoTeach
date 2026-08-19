function resultSet = attach_resampled_provenance_to_ik_result(resultSet, adapter)
%ATTACH_RESAMPLED_PROVENANCE_TO_IK_RESULT Decorate derived IK evidence.

    if ~isstruct(resultSet) || ~isfield(resultSet, 'segments') || ...
            ~isfield(resultSet, 'failures') || ~isstruct(adapter) || ...
            ~isfield(adapter, 'provenance_lookup')
        error('MonoTeach:InvalidResampledIKProvenanceInput', ...
            'IK result set and resampled adapter lookup are required.');
    end
    for segmentIndex = 1:numel(resultSet.segments)
        results = resultSet.segments(segmentIndex).results;
        for resultIndex = 1:numel(results)
            results(resultIndex).resampled_provenance = lookup( ...
                adapter.provenance_lookup, results(resultIndex).source_index);
        end
        resultSet.segments(segmentIndex).results = results;
    end
    for failureIndex = 1:numel(resultSet.failures)
        resultSet.failures(failureIndex).resampled_provenance = lookup( ...
            adapter.provenance_lookup, resultSet.failures(failureIndex).source_index);
    end
end


function value = lookup(values, resampledIndex)
    index = find([values.resampled_index] == resampledIndex, 1, 'first');
    if isempty(index)
        error('MonoTeach:MissingResampledIKProvenance', ...
            'No resampled provenance for solver source_index %d.', resampledIndex);
    end
    value = values(index);
end
