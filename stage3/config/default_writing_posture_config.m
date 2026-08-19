function config = default_writing_posture_config()
%DEFAULT_WRITING_POSTURE_CONFIG Geometry settings for a future writing posture.
%
% This configuration deliberately does not instantiate an IK solver or a
% constraint.  The final tool-facing plane-normal direction remains open
% until the +normal and -normal reachability experiments are complete.

    % Legacy robot body carrying the writing tool.
    config.end_effector = 'body5';

    % Local end-effector axis intended to aim toward the task plane.
    config.tool_axis = 'z';

    % Deliberately unresolved: do not select a final +normal/-normal policy
    % before the dedicated reachability experiment.
    config.plane_normal_sign = 'undecided';
    config.candidate_plane_normal_signs = [+1, -1];

    % Geometric distance used to construct an aiming ray [m].  This is not
    % the physical pen length and must not be interpreted as one.
    config.aim_distance_m = 0.050;

    % Intended Cartesian and angular tolerances for the later posture-aware
    % solver configuration.  They do not enable a solver in this step.
    config.position_tolerance_m = 1.0e-4;
    config.aiming_angular_tolerance_rad = deg2rad(5.0);

    % Keep future posture solving reproducible by default, consistent with
    % the accepted Stage 3.2 deterministic baseline.
    config.solver_algorithm = 'BFGSGradientProjection';
    config.allow_random_restart = false;
end
