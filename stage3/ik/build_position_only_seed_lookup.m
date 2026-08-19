function lookup = build_position_only_seed_lookup(positionResultSet)
%BUILD_POSITION_ONLY_SEED_LOOKUP Map accepted Stage 3.2 q values by source.
%
% Only accepted Position-Only success records enter the lookup.  The map is
% derived data: it neither changes the frozen Stage 3.2 solver nor mutates
% its result set.

    if ~isstruct(positionResultSet) || ~isfield(positionResultSet, 'segments')
        error('MonoTeach:InvalidPositionOnlySeedLookupInput', ...
            'Position-Only result set must contain segments.');
    end

    sourceIndices = zeros(1, 0);
    qRad = zeros(0, 5);
    for segmentIndex = 1:numel(positionResultSet.segments)
        results = positionResultSet.segments(segmentIndex).results;
        for resultIndex = 1:numel(results)
            result = results(resultIndex);
            if ~isfield(result, 'source_index') || ~isfield(result, 'q') || ...
                    ~isequal(size(result.q), [1, 5]) || ...
                    any(~isfinite(result.q))
                error('MonoTeach:InvalidPositionOnlySeedRecord', ...
                    'Accepted Position-Only seed records require finite 1-by-5 q.');
            end
            sourceIndices(end + 1) = result.source_index; %#ok<AGROW>
            qRad(end + 1, :) = result.q; %#ok<AGROW>
        end
    end

    if numel(unique(sourceIndices)) ~= numel(sourceIndices)
        error('MonoTeach:DuplicatePositionOnlySeedSourceIndex', ...
            'Position-Only seed lookup requires unique source_index values.');
    end
    lookup = struct();
    lookup.source_indices = sourceIndices;
    lookup.q_rad = qRad;
    lookup.provenance = 'same_source_position_only_q';
end
