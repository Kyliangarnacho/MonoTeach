function validate_workspace_trajectory(trajectory)
%VALIDATE_WORKSPACE_TRAJECTORY Validate the Stage 2.3 workspace JSON contract.
%
% Input:
%   trajectory - struct returned by jsondecode for schema_version 1.0
%
% Output:
%   None. Raises an error when required fields or sample semantics are invalid.
%
% This function is read-only: it does not add defaults or modify the decoded
% metadata or samples.

    root = require_struct_scalar(trajectory, 'root');
    require_fields(root, {'schema_version', 'metadata', 'samples'}, 'root');

    schemaVersion = root.schema_version;
    if ~is_nonempty_text(schemaVersion) || ~strcmp(string(schemaVersion), "1.0")
        error( ...
            'MonoTeach:UnsupportedWorkspaceTrajectorySchema', ...
            'schema_version must be the supported value "1.0".');
    end

    validate_metadata(root.metadata);
    validate_samples(root.samples);
end


function validate_metadata(metadata)

    metadata = require_struct_scalar(metadata, 'metadata');
    requiredFields = { ...
        'source_trajectory_id', ...
        'workspace_calibration_id', ...
        'coordinate_frame', ...
        'width_mm', ...
        'height_mm'};

    require_fields(metadata, requiredFields, 'metadata');

    require_nonempty_text( ...
        metadata.source_trajectory_id, ...
        'metadata.source_trajectory_id');

    require_nonempty_text( ...
        metadata.workspace_calibration_id, ...
        'metadata.workspace_calibration_id');

    require_nonempty_text( ...
        metadata.coordinate_frame, ...
        'metadata.coordinate_frame');

    if ~strcmp(string(metadata.coordinate_frame), "workspace_2d")
        error( ...
            'MonoTeach:InvalidWorkspaceCoordinateFrame', ...
            'metadata.coordinate_frame must be "workspace_2d".');
    end

    require_positive_finite_scalar(metadata.width_mm, 'metadata.width_mm');
    require_positive_finite_scalar(metadata.height_mm, 'metadata.height_mm');
end


function validate_samples(samples)

    if isempty(samples)
        return;
    end

    if ~isstruct(samples)
        error( ...
            'MonoTeach:InvalidWorkspaceTrajectorySamples', ...
            'samples must be a JSON array of objects.');
    end

    requiredFields = { ...
        't_ms', ...
        'valid', ...
        'x_mm', ...
        'y_mm', ...
        'inside_workspace', ...
        'invalid_reason'};

    for i = 1:numel(samples)
        sample = samples(i);
        context = sprintf('samples(%d)', i);

        require_fields(sample, requiredFields, context);
        require_nonnegative_finite_scalar(sample.t_ms, [context '.t_ms']);

        if ~islogical(sample.valid) || ~isscalar(sample.valid)
            error( ...
                'MonoTeach:InvalidWorkspaceTrajectoryValidFlag', ...
                '%s.valid must be a logical scalar.', ...
                context);
        end

        if ~islogical(sample.inside_workspace) || ...
                ~isscalar(sample.inside_workspace)
            error( ...
                'MonoTeach:InvalidWorkspaceTrajectoryInsideFlag', ...
                '%s.inside_workspace must be a logical scalar.', ...
                context);
        end

        if sample.valid
            require_finite_scalar(sample.x_mm, [context '.x_mm']);
            require_finite_scalar(sample.y_mm, [context '.y_mm']);

            if ~isempty(sample.invalid_reason)
                error( ...
                    'MonoTeach:ValidWorkspaceSampleHasInvalidReason', ...
                    '%s is valid and cannot include invalid_reason.', ...
                    context);
            end

        else
            if ~isempty(sample.x_mm) || ~isempty(sample.y_mm)
                error( ...
                    'MonoTeach:InvalidWorkspaceSampleHasCoordinates', ...
                    '%s is invalid and must not include x_mm or y_mm.', ...
                    context);
            end

            if sample.inside_workspace
                error( ...
                    'MonoTeach:InvalidWorkspaceSampleInside', ...
                    '%s is invalid and cannot be inside_workspace.', ...
                    context);
            end

            require_nonempty_text( ...
                sample.invalid_reason, ...
                [context '.invalid_reason']);
        end
    end
end


function value = require_struct_scalar(value, context)

    if ~isstruct(value) || ~isscalar(value)
        error( ...
            'MonoTeach:InvalidWorkspaceTrajectoryObject', ...
            '%s must be one JSON object.', ...
            context);
    end
end


function require_fields(data, requiredFields, context)

    missingFields = requiredFields(~isfield(data, requiredFields));

    if ~isempty(missingFields)
        error( ...
            'MonoTeach:MissingWorkspaceTrajectoryField', ...
            '%s is missing required fields: %s.', ...
            context, ...
            strjoin(missingFields, ', '));
    end
end


function require_nonempty_text(value, context)

    if ~is_nonempty_text(value)
        error( ...
            'MonoTeach:InvalidWorkspaceTrajectoryText', ...
            '%s must be a non-empty text value.', ...
            context);
    end
end


function result = is_nonempty_text(value)

    result = ...
        (ischar(value) && ~isempty(value)) || ...
        (isstring(value) && isscalar(value) && strlength(value) > 0);
end


function require_positive_finite_scalar(value, context)

    require_finite_scalar(value, context);

    if value <= 0
        error( ...
            'MonoTeach:InvalidWorkspaceTrajectoryDimension', ...
            '%s must be positive.', ...
            context);
    end
end


function require_nonnegative_finite_scalar(value, context)

    require_finite_scalar(value, context);

    if value < 0
        error( ...
            'MonoTeach:InvalidWorkspaceTrajectoryTimestamp', ...
            '%s must be non-negative.', ...
            context);
    end
end


function require_finite_scalar(value, context)

    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
            ~isfinite(value)
        error( ...
            'MonoTeach:InvalidWorkspaceTrajectoryNumber', ...
            '%s must be one finite numeric scalar.', ...
            context);
    end
end
