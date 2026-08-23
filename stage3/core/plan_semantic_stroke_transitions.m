function transitionSet = plan_semantic_stroke_transitions( ...
        semanticStrokeSet, taskPlaneConfig, transitionConfig)
%PLAN_SEMANTIC_STROKE_TRANSITIONS Build minimal pen-up lift/transfer/lower paths.
%
% This is a task-space transition artifact.  It is triggered only between
% adjacent semantic strokes, never between ordinary IK execution segments.
% Cartesian interpolation uses MATLAB transformtraj with fixed task-plane
% orientation.  No collision planner or robot execution connection is made.

    if nargin < 2 || isempty(taskPlaneConfig)
        taskPlaneConfig = default_task_plane_config();
    end
    if nargin < 3 || isempty(transitionConfig)
        transitionConfig = default_semantic_stroke_transition_config();
    end
    validate_inputs(semanticStrokeSet, taskPlaneConfig, transitionConfig);
    [normalBase, ~] = derive_task_plane_normal(taskPlaneConfig);
    rotation = taskPlaneConfig.T_base_taskplane(1:3, 1:3);

    transitions = repmat(empty_transition(), 1, 0);
    strokes = semanticStrokeSet.strokes;
    for index = 1:max(0, numel(strokes) - 1)
        strokeA = strokes(index);
        strokeB = strokes(index + 1);
        startXYZ = sample_xyz(strokeA.samples(end), index, 'end');
        targetXYZ = sample_xyz(strokeB.samples(1), index + 1, 'start');
        liftXYZ = startXYZ + transitionConfig.lift_distance_m * normalBase';
        transferXYZ = targetXYZ + transitionConfig.lift_distance_m * normalBase';
        [xyz, phase] = interpolate_legs( ...
            {startXYZ, liftXYZ, transferXYZ, targetXYZ}, ...
            rotation, transitionConfig.samples_per_leg);

        transition = empty_transition();
        transition.transition_index = index;
        transition.from_stroke_id = strokeA.stroke_id;
        transition.to_stroke_id = strokeB.stroke_id;
        transition.phase = phase;
        transition.xyz_m = xyz;
        transition.signed_normal_base = normalBase;
        transition.lift_distance_m = transitionConfig.lift_distance_m;
        transition.pen_state = repmat({'UP'}, size(xyz, 1), 1);
        transition.stroke_id = NaN(size(xyz, 1), 1);
        transitions(end + 1) = transition; %#ok<AGROW>
    end

    transitionSet = struct();
    transitionSet.artifact_type = 'SemanticStrokeTransitionSet';
    transitionSet.transition_method = 'transformtraj_lift_transfer_lower_v1';
    transitionSet.semantic_stroke_count = numel(strokes);
    transitionSet.transitions = transitions;
    transitionSet.summary = struct( ...
        'transition_count', numel(transitions), ...
        'all_pen_up', all(arrayfun(@(x) all(strcmp(x.pen_state, 'UP')), transitions)));
end


function [xyz, phases] = interpolate_legs(points, rotation, samplesPerLeg)

    xyz = zeros(0, 3);
    phases = cell(0, 1);
    phaseNames = {'lift', 'transfer', 'lower'};
    for leg = 1:3
        T0 = eye(4);
        TF = eye(4);
        T0(1:3, 1:3) = rotation;
        TF(1:3, 1:3) = rotation;
        T0(1:3, 4) = points{leg}(:);
        TF(1:3, 4) = points{leg + 1}(:);
        samples = linspace(0, 1, samplesPerLeg);
        transforms = transformtraj(T0, TF, [0, 1], samples);
        legXYZ = squeeze(transforms(1:3, 4, :))';
        if leg > 1
            legXYZ = legXYZ(2:end, :);
        end
        xyz = [xyz; legXYZ]; %#ok<AGROW>
        phaseValues = repmat(phaseNames(leg), size(legXYZ, 1), 1);
        phases = [phases; phaseValues]; %#ok<AGROW>
    end
end


function xyz = sample_xyz(sample, strokeIndex, label)

    if ~isstruct(sample) || ~all(isfield(sample, {'x_m', 'y_m', 'z_m'})) || ...
            any(~isfinite([sample.x_m, sample.y_m, sample.z_m]))
        error('MonoTeach:InvalidSemanticStrokeEndpoint', ...
            'Semantic stroke %d %s endpoint must have finite x_m/y_m/z_m.', ...
            strokeIndex, label);
    end
    xyz = [sample.x_m, sample.y_m, sample.z_m];
end


function transition = empty_transition()

    transition = struct('transition_index', NaN, 'from_stroke_id', NaN, ...
        'to_stroke_id', NaN, 'phase', {{}}, 'xyz_m', zeros(0, 3), ...
        'signed_normal_base', NaN(3, 1), 'lift_distance_m', NaN, ...
        'pen_state', {{}}, 'stroke_id', zeros(0, 1));
end


function validate_inputs(semanticStrokeSet, taskPlaneConfig, config)

    if ~isstruct(semanticStrokeSet) || ~isscalar(semanticStrokeSet) || ...
            ~isfield(semanticStrokeSet, 'strokes') || ...
            ~isstruct(semanticStrokeSet.strokes)
        error('MonoTeach:InvalidSemanticTransitionInput', ...
            'A SemanticStrokeSet is required.');
    end
    requiredPlane = {'T_base_taskplane'};
    if ~isstruct(taskPlaneConfig) || ~all(isfield(taskPlaneConfig, requiredPlane)) || ...
            ~isequal(size(taskPlaneConfig.T_base_taskplane), [4, 4])
        error('MonoTeach:InvalidSemanticTransitionPlane', ...
            'A valid Task Plane transform is required.');
    end
    if ~isstruct(config) || ~isscalar(config) || ...
            ~all(isfield(config, {'lift_distance_m', 'samples_per_leg'})) || ...
            ~isscalar(config.lift_distance_m) || config.lift_distance_m <= 0 || ...
            ~isscalar(config.samples_per_leg) || config.samples_per_leg < 2 || ...
            config.samples_per_leg ~= floor(config.samples_per_leg)
        error('MonoTeach:InvalidSemanticTransitionConfig', ...
            'Transition config needs positive lift distance and at least two samples per leg.');
    end
end
