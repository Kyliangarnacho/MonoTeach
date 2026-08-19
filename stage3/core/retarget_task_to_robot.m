function robotTask = retarget_task_to_robot(task, robotContext, taskPlaneConfig)
%RETARGET_TASK_TO_ROBOT Adapt canonical task data to one robot-base target set.
%
% The established workspace_to_task_trajectory mapping remains the sole
% mapping implementation.  This compatibility adapter only supplies its
% canonical workspace source and attaches RobotContext provenance afterwards.

    validate_inputs(task, robotContext);
    legacyCompatible = workspace_to_task_trajectory( ...
        task.source_workspace_trajectory, taskPlaneConfig);
    robotTask = legacyCompatible;
    robotTask.artifact_type = 'RobotTargetTrajectory';
    robotTask.robot_id = robotContext.id;
    robotTask.robot_backend = robotContext.backend;
    robotTask.task_coordinate_frame = task.coordinate_frame;
    robotTask.task_units = task.units;
    robotTask.task_barrier_semantics = task.barrier_semantics;
    robotTask.retargeting_method = ...
        'workspace_to_task_trajectory_legacy_compatibility_adapter';
end


function validate_inputs(task, context)

    if ~isstruct(task) || ~isscalar(task) || ...
            ~isfield(task, 'artifact_type') || ...
            ~strcmp(string(task.artifact_type), "CanonicalTaskTrajectory") || ...
            ~isfield(task, 'source_workspace_trajectory') || ...
            ~isstruct(context) || ~isscalar(context) || ...
            ~all(isfield(context, {'id', 'model', 'dof', 'end_effector', ...
            'joint_limits', 'backend'})) || context.dof < 1
        error('MonoTeach:InvalidRobotTaskRetargetInput', ...
            'Retargeting requires a canonical task and a complete RobotContext.');
    end
end
