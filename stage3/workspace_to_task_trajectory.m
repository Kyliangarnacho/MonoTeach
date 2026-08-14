function taskTrajectory = workspace_to_task_trajectory( ...
        workspaceTrajectory, ...
        config)
%WORKSPACE_TO_TASK_TRAJECTORY Map a workspace-mm trajectory into robot base m.
%
% Inputs:
%   workspaceTrajectory - validated Stage 2.3 workspace trajectory struct
%   config              - task-plane config from default_task_plane_config
%
% Output:
%   taskTrajectory      - new derived struct; valid samples have robot-base
%                         x_m/y_m/z_m, while invalid samples keep empty XYZ.
%
% This function performs only a geometric coordinate transform. It does not
% segment gaps, solve IK, animate the robot, or modify workspaceTrajectory.

    validate_workspace_trajectory(workspaceTrajectory);
    validate_task_plane_config(config);

    sourceSamples = workspaceTrajectory.samples;
    taskSamples = repmat(empty_task_sample(), size(sourceSamples));

    for i = 1:numel(sourceSamples)
        sourceSample = sourceSamples(i);

        % Keep the Stage 2.3 timing and validity evidence unchanged.
        taskSamples(i).t_ms = sourceSample.t_ms;
        taskSamples(i).valid = sourceSample.valid;
        taskSamples(i).inside_workspace = sourceSample.inside_workspace;
        taskSamples(i).invalid_reason = sourceSample.invalid_reason;

        if ~sourceSample.valid
            % Invalid gaps remain gaps in the derived base-frame trajectory.
            continue;
        end

        % Workspace [mm] -> centred task-plane [mm] -> task-plane [m].
        centredTaskPlaneM = ...
            config.scale * ( ...
            [sourceSample.x_mm, sourceSample.y_mm] - ...
            config.workspace_center_mm);

        % A workspace point lies on the task plane, so task-plane z is zero.
        taskPlanePointH = [centredTaskPlaneM, 0.0, 1.0]';

        % T_base_taskplane maps the homogeneous task-plane point into base [m].
        basePointH = config.T_base_taskplane * taskPlanePointH;

        taskSamples(i).x_m = basePointH(1);
        taskSamples(i).y_m = basePointH(2);
        taskSamples(i).z_m = basePointH(3);
    end

    % Copy source identity and add the coordinate-system evidence for this
    % derived data. workspaceTrajectory itself remains unchanged.
    taskMetadata = workspaceTrajectory.metadata;
    taskMetadata.source_coordinate_frame = ...
        workspaceTrajectory.metadata.coordinate_frame;
    taskMetadata.coordinate_frame = 'robot_base';
    taskMetadata.units = 'm';
    taskMetadata.workspace_center_mm = config.workspace_center_mm;
    taskMetadata.scale_m_per_mm = config.scale;
    taskMetadata.robot_anchor_m = config.robot_anchor_m;

    taskTrajectory = struct();
    taskTrajectory.source_schema_version = workspaceTrajectory.schema_version;
    taskTrajectory.coordinate_frame = 'robot_base';
    taskTrajectory.metadata = taskMetadata;
    taskTrajectory.samples = taskSamples;
end


function sample = empty_task_sample()

    sample = struct( ...
        't_ms', [], ...
        'valid', [], ...
        'inside_workspace', [], ...
        'invalid_reason', [], ...
        'x_m', [], ...
        'y_m', [], ...
        'z_m', []);
end


function validate_task_plane_config(config)

    if ~isstruct(config) || ~isscalar(config)
        error( ...
            'MonoTeach:InvalidTaskPlaneConfig', ...
            'config must be one task-plane configuration struct.');
    end

    requiredFields = { ...
        'workspace_center_mm', ...
        'scale', ...
        'robot_anchor_m', ...
        'T_base_taskplane'};

    missingFields = requiredFields(~isfield(config, requiredFields));

    if ~isempty(missingFields)
        error( ...
            'MonoTeach:MissingTaskPlaneConfigField', ...
            'config is missing required fields: %s.', ...
            strjoin(missingFields, ', '));
    end

    require_finite_vector( ...
        config.workspace_center_mm, ...
        2, ...
        'config.workspace_center_mm');

    if ~isnumeric(config.scale) || ~isscalar(config.scale) || ...
            ~isreal(config.scale) || ~isfinite(config.scale) || ...
            config.scale <= 0
        error( ...
            'MonoTeach:InvalidTaskPlaneScale', ...
            'config.scale must be one positive finite metres-per-millimetre value.');
    end

    require_finite_vector( ...
        config.robot_anchor_m, ...
        3, ...
        'config.robot_anchor_m');

    T = config.T_base_taskplane;
    if ~isnumeric(T) || ~isequal(size(T), [4, 4]) || ...
            ~isreal(T) || any(~isfinite(T), 'all')
        error( ...
            'MonoTeach:InvalidTaskPlaneTransform', ...
            'config.T_base_taskplane must be one finite 4-by-4 matrix.');
    end

    if norm(T(4, :) - [0, 0, 0, 1]) > 1e-12
        error( ...
            'MonoTeach:InvalidTaskPlaneTransform', ...
            'config.T_base_taskplane must use homogeneous final row [0 0 0 1].');
    end

    if norm(T(1:3, 4)' - config.robot_anchor_m) > 1e-12
        error( ...
            'MonoTeach:InconsistentTaskPlaneAnchor', ...
            'config.robot_anchor_m must equal T_base_taskplane translation.');
    end
end


function require_finite_vector(value, expectedLength, context)

    if ~isnumeric(value) || ~isreal(value) || ...
            ~isequal(size(value), [1, expectedLength]) || ...
            any(~isfinite(value), 'all')
        error( ...
            'MonoTeach:InvalidTaskPlaneConfig', ...
            '%s must be one finite 1-by-%d numeric vector.', ...
            context, ...
            expectedLength);
    end
end
