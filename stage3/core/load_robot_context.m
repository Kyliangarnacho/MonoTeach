function context = load_robot_context(robotId)
%LOAD_ROBOT_CONTEXT Load one supported robot's minimal algorithm contract.
%
% The first version deliberately supports only the existing Legacy 5DOF
% MATLAB rigidBodyTree backend.  It creates an adapter boundary; it does not
% claim cross-robot support or replace build_legacy_robot.

    if nargin < 1 || isempty(robotId)
        robotId = "legacy5";
    end
    robotId = lower(string(robotId));
    if ~isscalar(robotId) || robotId ~= "legacy5"
        error('MonoTeach:UnsupportedRobotContext', ...
            'Only RobotContext "legacy5" is supported in this version.');
    end

    model = build_legacy_robot();
    dof = numel(homeConfiguration(model));
    jointNames = strings(1, dof);
    jointLimits = zeros(dof, 2);
    for index = 1:dof
        jointNames(index) = string(model.Bodies{index}.Joint.Name);
        jointLimits(index, :) = model.Bodies{index}.Joint.PositionLimits;
    end

    context = struct();
    context.id = char(robotId);
    context.model = model;
    context.dof = dof;
    context.joint_names = cellstr(jointNames);
    context.end_effector = 'body5';
    context.home_q = homeConfiguration(model);
    context.joint_limits = jointLimits;
    % These are existing Stage 3.3 planning defaults, not verified actuator
    % ratings.  Keeping their source in the context lets timing stay N-DOF.
    context.velocity_limits = 0.5 * ones(1, dof);
    context.acceleration_limits = 2.0 * ones(1, dof);
    context.limit_source = 'stage3_3_planning_defaults_not_hardware_ratings';
    context.tool = struct('axis', 'z', 'frame', 'end_effector_local');
    context.capabilities = struct('position_only_ik', true, ...
        'tool_z_aiming', true, 'free_space_planning', false);
    context.backend = 'matlab_rigidbodytree';
end
