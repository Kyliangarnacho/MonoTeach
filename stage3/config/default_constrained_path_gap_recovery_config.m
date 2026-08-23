function config = default_constrained_path_gap_recovery_config(postureConfig)
%DEFAULT_CONSTRAINED_PATH_GAP_RECOVERY_CONFIG Frozen V1 orientation ladder.
%
% Recovery never relaxes target XYZ or the frozen Writing position
% tolerance.  Every eligible execution gap shares this deterministic
% direction-tolerance ladder; it is not tuned per trajectory or gap.

    if nargin < 1 || isempty(postureConfig)
        postureConfig = default_writing_posture_config();
    end
    validate_posture_config(postureConfig);

    strictDeg = rad2deg(postureConfig.aiming_angular_tolerance_rad);
    config = struct();
    config.artifact_type = 'ConstrainedPathGapRecoveryConfig';
    config.strict_direction_tolerance_deg = strictDeg;
    config.max_recovery_direction_tolerance_deg = 30.0;
    config.direction_tolerance_step_deg = 5.0;
    config.direction_tolerance_ladder_deg = strictDeg: ...
        config.direction_tolerance_step_deg: ...
        config.max_recovery_direction_tolerance_deg;
    config.position_tolerance_m = postureConfig.position_tolerance_m;
    config.solver_algorithm = postureConfig.solver_algorithm;
    config.allow_random_restart = false;
    config.duplicate_seed_tolerance_rad = 1.0e-12;
    config.recovery_reason = 'orientation_constraint_relaxation';
end


function validate_posture_config(postureConfig)

    if ~isstruct(postureConfig) || ...
            ~isfield(postureConfig, 'aiming_angular_tolerance_rad') || ...
            ~isfield(postureConfig, 'position_tolerance_m') || ...
            ~isfield(postureConfig, 'solver_algorithm') || ...
            ~isfinite(postureConfig.aiming_angular_tolerance_rad) || ...
            postureConfig.aiming_angular_tolerance_rad <= 0 || ...
            ~isfinite(postureConfig.position_tolerance_m) || ...
            postureConfig.position_tolerance_m <= 0 || ...
            postureConfig.allow_random_restart
        error('MonoTeach:InvalidGapRecoveryPostureConfig', ...
            'Gap recovery requires the frozen deterministic Writing posture config.');
    end
end
