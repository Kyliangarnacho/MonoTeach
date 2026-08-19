# Minimal robot API boundary

MonoTeach currently formally supports only `legacy5`, through MATLAB
`rigidBodyTree`.  The following boundary is intentionally small:

```text
Camera / WorkspaceTrajectory
        ↓
CanonicalTaskTrajectory (workspace frame, no joints)
        ↓
retarget_task_to_robot(task, RobotContext, placement)
        ↓
RobotTargetTrajectory (robot-base targets)
        ↓
IK / constrained path
        ↓
future free-space transition planning
        ↓
timing / execution
```

`load_robot_context("legacy5")` returns a struct with the model, DOF, joint
names, end effector, home configuration, joint limits, Stage 3.3 planning
velocity/acceleration limits, tool convention, capabilities, and backend.
The planning limits are not hardware ratings.

`workspace_to_task_trajectory` remains the validated Legacy compatibility
mapping.  `workspace_to_canonical_task` and `retarget_task_to_robot` create a
new explicit task-to-robot boundary without copying its mapping mathematics.

No MoveIt, ROS, URDF/SRDF, collision planner, or second robot is integrated.
Adding another robot requires a verified `RobotContext`, a robot-specific
placement/retargeting configuration, and context-aware IK/Writing adapter
validation.  It must not be inferred from this boundary alone.
