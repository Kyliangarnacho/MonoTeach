function result = ik_result_contract( ...
        targetXYZM, ...
        seedQ, ...
        q, ...
        solverStatus, ...
        poseErrorNorm, ...
        fkXYZM, ...
        jointLimitsRad)
%IK_RESULT_CONTRACT Build the canonical result for one Position-Only IK solve.
%
% The first nine fields are the stable per-target IK contract shared by the
% anchor baseline and a later continuous solver. source_index, t_ms, and
% delta_q deliberately start empty: they add trajectory provenance and
% adjacency evidence later without changing the single-point result shape.
%
% joint_limit_margin is the element-wise distance in radians to the closer
% lower or upper joint bound. A positive margin means the solution is inside
% that joint's limits; a negative margin makes an out-of-limit result explicit.

    require_finite_row_vector(targetXYZM, 3, 'targetXYZM');
    validate_configuration_pair(seedQ, q);
    require_finite_row_vector(fkXYZM, 3, 'fkXYZM');
    validate_joint_limits(jointLimitsRad, numel(q));
    validate_optional_pose_error(poseErrorNorm);

    if ~(ischar(solverStatus) || ...
            (isstring(solverStatus) && isscalar(solverStatus)))
        error( ...
            'MonoTeach:InvalidIKResultStatus', ...
            'solverStatus must be one character vector or string scalar.');
    end

    jointLimitMargin = min( ...
        q - jointLimitsRad(:, 1)', ...
        jointLimitsRad(:, 2)' - q);
    limitComparisonTolerance = 1.0e-12;
    withinJointLimits = all( ...
        q >= jointLimitsRad(:, 1)' - limitComparisonTolerance & ...
        q <= jointLimitsRad(:, 2)' + limitComparisonTolerance);

    result = struct();
    result.target_xyz_m = targetXYZM;
    result.seed_q = seedQ;
    result.q = q;
    result.solver_status = solverStatus;
    result.pose_error_norm = poseErrorNorm;
    result.fk_xyz_m = fkXYZM;
    result.position_error_m = norm(fkXYZM - targetXYZM);
    result.within_joint_limits = withinJointLimits;
    result.joint_limit_margin = jointLimitMargin;

    % These stay empty for the anchor baseline. A future continuous IK solver
    % can fill them per derived TaskTrajectory sample without inventing a
    % second, incompatible IK result representation.
    result.source_index = [];
    result.t_ms = [];
    result.delta_q = [];
end


function validate_configuration_pair(seedQ, q)

    if ~isnumeric(seedQ) || ~isreal(seedQ) || ...
            ~isrow(seedQ) || isempty(seedQ) || any(~isfinite(seedQ), 'all')
        error( ...
            'MonoTeach:InvalidIKResultSeed', ...
            'seedQ must be one nonempty finite real row vector.');
    end

    if ~isnumeric(q) || ~isreal(q) || ...
            ~isrow(q) || ~isequal(size(q), size(seedQ)) || ...
            any(~isfinite(q), 'all')
        error( ...
            'MonoTeach:InvalidIKResultConfiguration', ...
            'q must be a finite real row vector with the same size as seedQ.');
    end
end


function validate_joint_limits(limits, dof)

    if ~isnumeric(limits) || ~isreal(limits) || ...
            ~isequal(size(limits), [dof, 2]) || ...
            any(~isfinite(limits), 'all') || ...
            any(limits(:, 1) > limits(:, 2))
        error( ...
            'MonoTeach:InvalidIKResultJointLimits', ...
            'jointLimitsRad must be finite dof-by-2 lower/upper limits.');
    end
end


function validate_optional_pose_error(value)

    if isempty(value)
        return;
    end

    if ~isnumeric(value) || ~isscalar(value) || ~isreal(value) || ...
            ~isfinite(value) || value < 0
        error( ...
            'MonoTeach:InvalidIKResultPoseError', ...
            'poseErrorNorm must be empty or one finite nonnegative scalar.');
    end
end


function require_finite_row_vector(value, expectedLength, context)

    if ~isnumeric(value) || ~isreal(value) || ...
            ~isequal(size(value), [1, expectedLength]) || ...
            any(~isfinite(value), 'all')
        error( ...
            'MonoTeach:InvalidIKResultInput', ...
            '%s must be one finite 1-by-%d numeric vector.', ...
            context, ...
            expectedLength);
    end
end
