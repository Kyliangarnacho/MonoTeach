function [semanticSamples, semantics] = normalize_task_semantics(samples)
%NORMALIZE_TASK_SEMANTICS Attach explicit pen-state semantics to task samples.
%
% Existing Stage 2/3 workspace samples predate pen state.  For that legacy
% input only, each valid sample is explicitly labelled DOWN/stroke 1 and an
% invalid gap is labelled UP/no stroke.  This is a compatibility convention,
% not visual pen-state recognition or inference.  Future producers may supply
% pen_state (DOWN or UP) and stroke_id directly; both fields must then exist.

    if ~isstruct(samples)
        error('MonoTeach:InvalidTaskSemanticSamples', ...
            'Task semantic samples must be a struct array.');
    end

    semanticSamples = samples;
    if isempty(samples)
        semantics = semantic_metadata('legacy_single_stroke_default', true);
        return;
    end

    hasPenState = isfield(samples, 'pen_state');
    hasStrokeId = isfield(samples, 'stroke_id');
    if hasPenState ~= hasStrokeId
        error('MonoTeach:IncompleteTaskSemantics', ...
            'pen_state and stroke_id must be supplied together.');
    end

    if ~hasPenState
        for i = 1:numel(semanticSamples)
            if ~isfield(semanticSamples(i), 'valid') || ...
                    ~islogical(semanticSamples(i).valid) || ...
                    ~isscalar(semanticSamples(i).valid)
                error('MonoTeach:InvalidTaskSemanticSamples', ...
                    'Legacy task samples must contain logical scalar valid flags.');
            end

            if semanticSamples(i).valid
                semanticSamples(i).pen_state = 'DOWN';
                semanticSamples(i).stroke_id = 1;
            else
                semanticSamples(i).pen_state = 'UP';
                semanticSamples(i).stroke_id = NaN;
            end
        end
        semantics = semantic_metadata('legacy_single_stroke_default', true);
        return;
    end

    for i = 1:numel(semanticSamples)
        [penState, strokeId] = validate_explicit_semantics( ...
            semanticSamples(i).pen_state, semanticSamples(i).stroke_id, i);
        semanticSamples(i).pen_state = penState;
        semanticSamples(i).stroke_id = strokeId;
    end
    semantics = semantic_metadata('explicit_input', false);
end


function [penState, strokeId] = validate_explicit_semantics(rawPenState, rawStrokeId, index)

    penState = upper(string(rawPenState));
    if ~isscalar(penState) || ~ismember(penState, ["DOWN", "UP"])
        error('MonoTeach:InvalidPenState', ...
            'samples(%d).pen_state must be DOWN or UP.', index);
    end
    penState = char(penState);

    if ~isnumeric(rawStrokeId) || ~isscalar(rawStrokeId) || ...
            ~isreal(rawStrokeId)
        error('MonoTeach:InvalidStrokeId', ...
            'samples(%d).stroke_id must be one real numeric scalar.', index);
    end

    if strcmp(penState, 'DOWN')
        if ~isfinite(rawStrokeId) || rawStrokeId <= 0 || ...
                rawStrokeId ~= floor(rawStrokeId)
            error('MonoTeach:InvalidStrokeId', ...
                'DOWN samples(%d).stroke_id must be a positive integer.', index);
        end
    elseif ~isnan(rawStrokeId)
        error('MonoTeach:InvalidStrokeId', ...
            'UP samples(%d).stroke_id must be NaN (no semantic stroke).', index);
    end
    strokeId = rawStrokeId;
end


function semantics = semantic_metadata(source, legacyDefaultApplied)

    semantics = struct( ...
        'pen_state_contract', 'DOWN_or_UP', ...
        'stroke_id_contract', 'positive_integer_for_DOWN_NaN_for_UP', ...
        'semantic_source', source, ...
        'legacy_default_applied', legacyDefaultApplied);
end
