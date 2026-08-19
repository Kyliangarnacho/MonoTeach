function [nPlane, candidateNormals] = derive_task_plane_normal(taskPlaneConfig)
%DERIVE_TASK_PLANE_NORMAL Derive task-plane normals in the robot base frame.
%
% Inputs:
%   taskPlaneConfig - configuration containing T_base_taskplane.
%
% Outputs:
%   nPlane           - unit base-frame normal for the task-plane +Z axis.
%   candidateNormals - [nPlane, -nPlane] for later posture experiments.
%
% The task-plane normal is derived from the transform rotation, rather than
% hard-coding a base-frame direction:
%   R = T_base_taskplane(1:3, 1:3);
%   nPlane = R(:, 3);

    if ~isstruct(taskPlaneConfig) || ...
            ~isfield(taskPlaneConfig, 'T_base_taskplane')
        error('MonoTeach:TaskPlaneNormal:MissingTransform', ...
            'taskPlaneConfig must contain T_base_taskplane.');
    end

    TBaseTaskPlane = taskPlaneConfig.T_base_taskplane;
    if ~isnumeric(TBaseTaskPlane) || ~isequal(size(TBaseTaskPlane), [4, 4])
        error('MonoTeach:TaskPlaneNormal:InvalidTransformSize', ...
            'T_base_taskplane must be a numeric 4-by-4 matrix.');
    end

    R = TBaseTaskPlane(1:3, 1:3);
    nPlane = R(:, 3);
    if ~isreal(nPlane) || ~all(isfinite(nPlane))
        error('MonoTeach:TaskPlaneNormal:NonFiniteNormal', ...
            'T_base_taskplane rotation third column must be finite and real.');
    end

    normalMagnitude = norm(nPlane);
    if ~isfinite(normalMagnitude) || normalMagnitude <= eps
        error('MonoTeach:TaskPlaneNormal:DegenerateNormal', ...
            'T_base_taskplane rotation third column must have nonzero length.');
    end

    nPlane = nPlane / normalMagnitude;
    candidateNormals = [nPlane, -nPlane];
end
