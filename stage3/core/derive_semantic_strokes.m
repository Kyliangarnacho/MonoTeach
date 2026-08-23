function semanticStrokeSet = derive_semantic_strokes(taskTrajectory)
%DERIVE_SEMANTIC_STROKES Group explicit DOWN samples without considering IK.
%
% Semantic strokes are task-level pen-state runs.  They are intentionally
% independent from pre-IK eligibility and IK-success execution segments: an
% IK failure never changes stroke_id, and an UP sample always ends a semantic
% stroke even if adjacent DOWN samples would otherwise be executable.

    validate_task_semantics(taskTrajectory);
    samples = taskTrajectory.samples;
    strokes = repmat(empty_stroke(), 1, 0);
    active = false;

    for i = 1:numel(samples)
        isDown = strcmp(string(samples(i).pen_state), "DOWN");
        currentStrokeId = samples(i).stroke_id;

        if isDown && (~active || currentStrokeId ~= activeStrokeId)
            if active
                strokes(end + 1) = finish_stroke( ...
                    activeStrokeId, activeIndices, samples(activeIndices)); %#ok<AGROW>
                strokes(end).semantic_stroke_index = numel(strokes);
            end
            active = true;
            activeStrokeId = currentStrokeId;
            activeIndices = i;
        elseif isDown
            activeIndices(end + 1) = i; %#ok<AGROW>
        elseif active
            strokes(end + 1) = finish_stroke( ...
                activeStrokeId, activeIndices, samples(activeIndices)); %#ok<AGROW>
            strokes(end).semantic_stroke_index = numel(strokes);
            active = false;
        end
    end

    if active
        strokes(end + 1) = finish_stroke( ...
            activeStrokeId, activeIndices, samples(activeIndices)); %#ok<AGROW>
        strokes(end).semantic_stroke_index = numel(strokes);
    end

    semanticStrokeSet = struct();
    semanticStrokeSet.artifact_type = 'SemanticStrokeSet';
    if isfield(taskTrajectory, 'artifact_type')
        semanticStrokeSet.source_artifact_type = taskTrajectory.artifact_type;
    else
        semanticStrokeSet.source_artifact_type = 'TaskTrajectory';
    end
    semanticStrokeSet.strokes = strokes;
    semanticStrokeSet.summary = struct( ...
        'semantic_stroke_count', numel(strokes), ...
        'down_sample_count', sum(strcmp(string({samples.pen_state}), "DOWN")), ...
        'up_sample_count', sum(strcmp(string({samples.pen_state}), "UP")));
end


function stroke = finish_stroke(strokeId, sourceIndices, samples)

    stroke = struct( ...
        'semantic_stroke_index', [], ...
        'stroke_id', strokeId, ...
        'source_indices', sourceIndices, ...
        'samples', samples);
    stroke.semantic_stroke_index = stroke_id_placeholder();
end


function value = stroke_id_placeholder()

% The caller replaces this deterministic placeholder after the struct is
% appended, keeping the per-stroke constructor concise.
    value = NaN;
end


function stroke = empty_stroke()

    stroke = struct( ...
        'semantic_stroke_index', [], ...
        'stroke_id', [], ...
        'source_indices', [], ...
        'samples', []);
end


function validate_task_semantics(taskTrajectory)

    if ~isstruct(taskTrajectory) || ~isscalar(taskTrajectory) || ...
            ~isfield(taskTrajectory, 'samples') || ...
            ~isstruct(taskTrajectory.samples)
        error('MonoTeach:InvalidSemanticStrokeInput', ...
            'A task trajectory with semantic samples is required.');
    end

    requiredFields = {'pen_state', 'stroke_id'};
    samples = taskTrajectory.samples;
    for i = 1:numel(samples)
        if ~all(isfield(samples(i), requiredFields))
            error('MonoTeach:MissingTaskSemantics', ...
                'samples(%d) must contain pen_state and stroke_id.', i);
        end
        normalize_task_semantics(samples(i));
    end
end
