function config = default_ik_config()
%DEFAULT_IK_CONFIG Default deterministic Position-Only IK baseline settings.
%
% This Stage 3.2A configuration intentionally solves just one fixed
% robot_anchor target. It does not define continuous-segment policy,
% previous-q seeds, retry policy, or robot execution behavior.

    config.end_effector = 'body5';

    % inverseKinematics weights are [orientation_x orientation_y
    % orientation_z position_x position_y position_z].  A Legacy 5DOF arm
    % cannot generally satisfy arbitrary full poses, so this baseline tracks
    % only target translation and leaves all orientation weights at zero.
    % This is not a final writing/tool-posture strategy: tip inclination and
    % roll must be designed and validated separately before robot execution.
    config.weights = [0, 0, 0, 1, 1, 1];

    % Independent FK validation threshold for the target translation [m].
    config.position_error_tolerance_m = 1.0e-4;

    % Keep one reproducible homeConfiguration-seeded solve for the baseline.
    % Random restarts may be useful in a later recovery policy, but would hide
    % the deterministic first-attempt behavior measured here.
    config.allow_random_restart = false;

    % R2024a inverseKinematics supports this named solver. Its actual
    % SolverParameters are read at runtime before AllowRandomRestart is set.
    config.solver_algorithm = 'BFGSGradientProjection';
end
