function seeds = build_writing_recovery_ladder( ...
        previousSuccessQ, positionOnlySeedLookup, sourceIndex, centerSeed, ...
        homeQ, recoveryPolicy)
%BUILD_WRITING_RECOVERY_LADDER Ordered, deduplicated deterministic seeds.
%
% An unavailable previous or same-source Position-Only q is skipped.  The
% first provenance wins if two seed vectors are identical within tolerance.

    validate_inputs(centerSeed, homeQ, recoveryPolicy);
    seeds = repmat(struct('provenance', '', 'q', NaN(1, 5)), 1, 0);
    for sourceIndexInPolicy = 1:numel(recoveryPolicy.seed_sources)
        provenance = recoveryPolicy.seed_sources{sourceIndexInPolicy};
        q = resolve_seed_q( ...
            provenance, previousSuccessQ, positionOnlySeedLookup, sourceIndex, ...
            centerSeed, homeQ);
        if isempty(q)
            continue;
        end
        q = reshape(q, 1, []);
        if any(~isfinite(q)) || ~isequal(size(q), [1, 5])
            error('MonoTeach:InvalidWritingRecoverySeed', ...
                'Recovery seeds must be finite 1-by-5 configurations.');
        end
        if ~isempty(seeds) && any(arrayfun(@(seed) ...
                norm(seed.q - q) <= recoveryPolicy.duplicate_seed_tolerance_rad, ...
                seeds))
            continue;
        end
        seeds(end + 1) = struct('provenance', provenance, 'q', q); %#ok<AGROW>
    end
end


function q = resolve_seed_q( ...
        provenance, previousSuccessQ, lookup, sourceIndex, centerSeed, homeQ)

    q = [];
    switch provenance
        case 'previous_success_q'
            q = previousSuccessQ;
        case 'same_source_position_only_q'
            if isempty(lookup) || isempty(lookup.source_indices)
                return;
            end
            match = find(lookup.source_indices == sourceIndex, 1, 'first');
            if ~isempty(match)
                q = lookup.q_rad(match, :);
            end
        case 'candidate_center_writing_q'
            q = centerSeed.q;
        case 'home_configuration'
            q = homeQ;
        otherwise
            error('MonoTeach:UnknownWritingRecoverySeedSource', ...
                'Unsupported deterministic seed source: %s', provenance);
    end
end


function validate_inputs(centerSeed, homeQ, recoveryPolicy)

    if ~isstruct(centerSeed) || ~isfield(centerSeed, 'q') || ...
            ~isequal(size(centerSeed.q), [1, 5]) || ...
            ~isequal(size(homeQ), [1, 5]) || ...
            ~isstruct(recoveryPolicy) || ~isfield(recoveryPolicy, 'seed_sources') || ...
            ~isfield(recoveryPolicy, 'duplicate_seed_tolerance_rad')
        error('MonoTeach:InvalidWritingRecoveryLadderInput', ...
            'Recovery ladder inputs are incomplete.');
    end
end
