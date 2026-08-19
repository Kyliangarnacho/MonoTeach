function config = default_task_plane_config()
%DEFAULT_TASK_PLANE_CONFIG Default simulation-only WorkspaceTrajectory layout.
%
% The task plane is centred on the Stage 2.3 workspace centre. Before applying
% T_base_taskplane, convert a workspace point [x_mm, y_mm] to centred task-plane
% metres: scale * ([x_mm, y_mm] - workspace_center_mm).

    % Stage 2.3 workspace dimensions and its image/workspace centre [mm].
    config.workspace_width_mm = 190.0;
    config.workspace_height_mm = 290.0;
    config.workspace_center_mm = [95.0, 145.0];

    % Uniform visual retargeting scale [base metres per workspace millimetre].
    % The resulting rectangle is 0.095 m by 0.145 m.
    config.scale = 5.0e-4;

    % Location of the workspace centre in the Legacy robot base frame [m].
    % This is a simulation anchor, not a calibrated physical robot measurement.
    config.robot_anchor_m = [0.100, 0.040, 0.255];

    % Homogeneous transform from centred task-plane metres to robot base metres.
    % Task +X -> base +X, task +Y -> base -Z, task +Z (plane normal) -> base +Y.
    % Translation is robot_anchor_m, so the centred task-plane origin is the
    % workspace centre in the base coordinate system.
    config.T_base_taskplane = [ ...
        1,  0,  0, config.robot_anchor_m(1); ...
        0,  0,  1, config.robot_anchor_m(2); ...
        0, -1,  0, config.robot_anchor_m(3); ...
        0,  0,  0, 1];
end
