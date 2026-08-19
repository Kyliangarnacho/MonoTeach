function seed = resolve_writing_candidate_center_seed( ...
        robot, candidateTaskPlaneConfig, ikConfig, writingPostureConfig, ...
        candidateConfig)
%RESOLVE_WRITING_CANDIDATE_CENTER_SEED Derive accepted centre writing seed.
%
% This repeats the accepted centre feasibility check with BFGS and no random
% samples.  The returned q is accepted by independent FK, direction, and
% joint-limit checks before it is used as a continuous-writing restart seed.

    validate_inputs(robot, candidateTaskPlaneConfig, candidateConfig);

    postureConfig = writingPostureConfig;
    postureConfig.candidate_plane_normal_signs = candidateConfig.normal_sign;
    diagnosticConfig = default_writing_anchor_diagnostic_config();
    diagnosticConfig.solver_algorithms = {'BFGSGradientProjection'};
    diagnosticConfig.random_seed_count = 0;
    diagnosticConfig.allow_random_restart = false;
    diagnostic = diagnose_writing_anchor_multistart( ...
        robot, candidateTaskPlaneConfig, ikConfig, postureConfig, diagnosticConfig);
    run = diagnostic.runs(1);
    acceptedTrialIndex = find([run.trials.accepted], 1, 'first');
    if isempty(acceptedTrialIndex)
        error( ...
            'MonoTeach:WritingCandidateCenterNotAccepted', ...
            ['The approved candidate centre did not retain an accepted ' ...
             'writing configuration.']);
    end

    trial = run.trials(acceptedTrialIndex);
    seed = struct();
    seed.q = trial.candidate_q;
    seed.provenance = 'candidate_center_accepted_writing_q';
    seed.target_xyz_m = diagnostic.target_xyz_m;
    seed.signed_normal = run.signed_normal;
    seed.position_error_m = trial.position_error_m;
    seed.direction_error_rad = trial.tool_direction_error_rad;
    seed.direction_error_deg = trial.tool_direction_error_deg;
    seed.joint_limit_margin = trial.joint_limit_margin;
    seed.solver_status = trial.solver_status;
end


function validate_inputs(robot, taskPlaneConfig, candidateConfig)

    if ~isa(robot, 'rigidBodyTree') || ~strcmp(robot.DataFormat, 'row') || ...
            robot.NumBodies ~= 5 || ...
            ~isequal(taskPlaneConfig.robot_anchor_m, candidateConfig.center_m)
        error( ...
            'MonoTeach:InvalidWritingCenterSeedInput', ...
            'Center seed requires the approved row-format 5DOF candidate.');
    end
end
