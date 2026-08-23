function task = workspace_to_canonical_task(workspaceTrajectory)
%WORKSPACE_TO_CANONICAL_TASK Preserve a robot-independent demonstrated task.
%
% This thin derived artifact retains the immutable WorkspaceTrajectory points,
% validity/inside-workspace barriers, and provenance in workspace millimetres.
% It intentionally contains no robot-base coordinates and no joint states.

    validate_workspace_trajectory(workspaceTrajectory);
    [semanticSamples, semanticContract] = ...
        normalize_task_semantics(workspaceTrajectory.samples);
    task = struct();
    task.artifact_type = 'CanonicalTaskTrajectory';
    task.coordinate_frame = workspaceTrajectory.metadata.coordinate_frame;
    task.units = 'mm';
    task.source_schema_version = workspaceTrajectory.schema_version;
    task.metadata = workspaceTrajectory.metadata;
    task.metadata.task_semantics = semanticContract;
    task.samples = semanticSamples;
    task.semantic_stroke_contract = semanticContract;
    task.barrier_semantics = 'invalid_or_outside_workspace_is_not_interpolated';
    task.source_workspace_trajectory = workspaceTrajectory;
end


function validate_workspace_trajectory(trajectory)

    required = {'schema_version', 'metadata', 'samples'};
    if ~isstruct(trajectory) || ~isscalar(trajectory) || ...
            ~all(isfield(trajectory, required)) || ~isstruct(trajectory.metadata) || ...
            ~isstruct(trajectory.samples) || ...
            ~isfield(trajectory.metadata, 'coordinate_frame') || ...
            ~contains(lower(string(trajectory.metadata.coordinate_frame)), "workspace")
        error('MonoTeach:InvalidCanonicalTaskInput', ...
            'Canonical task input must be one validated workspace trajectory.');
    end
end
