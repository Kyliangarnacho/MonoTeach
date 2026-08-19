function diagnostic = diagnose_writing_task_dof( ...
        robot, taskPlaneConfig, ikConfig, writingPostureConfig)
%DIAGNOSE_WRITING_TASK_DOF Diagnose local DOF contribution at q_anchor.
%
% The R2024a geometricJacobian output is verified against local FK finite
% differences before this helper is used: rows 1:3 are base-frame angular
% velocity and rows 4:6 are base-frame linear velocity. This helper does not
% modify the robot, anchor, WritingPostureConfig, or any solver/constraint.

    validate_inputs(robot, writingPostureConfig);

    endEffector = char(string(writingPostureConfig.end_effector));
    anchorResult = solve_anchor_ik(robot, taskPlaneConfig, ikConfig);
    qAnchor = anchorResult.q;
    body5Pose = getTransform(robot, qAnchor, endEffector);
    toolZ = body5Pose(1:3, 3);

    % R2024a geometricJacobian is 6-by-n: [angular; linear], base frame.
    J = geometricJacobian(robot, qAnchor, endEffector);
    angularJacobian = J(1:3, :);
    linearJacobian = J(4:6, :);

    finiteDifference = j5_finite_difference( ...
        robot, qAnchor, endEffector, body5Pose);

    % A unit direction has two local degrees of freedom. Use the current
    % body5 X/Y axes as an orthonormal tangent basis to tool Z, then project
    % d(toolZ)/dq = omega x toolZ into those two independent coordinates.
    toolTangentBasis = body5Pose(1:3, 1:2);
    toolDirectionJacobian3 = -skew_symmetric(toolZ) * angularJacobian;
    toolDirectionJacobian2 = toolTangentBasis' * toolDirectionJacobian3;
    writingTaskJacobian = [linearJacobian; toolDirectionJacobian2];
    singularValues = svd(writingTaskJacobian);
    defaultRankTolerance = max(size(writingTaskJacobian)) * ...
        eps(max(singularValues));

    diagnostic = struct();
    diagnostic.q_anchor = qAnchor;
    diagnostic.body5_pose = body5Pose;
    diagnostic.jacobian = J;
    diagnostic.jacobian_shape = size(J);
    diagnostic.angular_contribution = angularJacobian;
    diagnostic.linear_contribution = linearJacobian;
    diagnostic.j5_angular_contribution = angularJacobian(:, 5);
    diagnostic.j5_linear_contribution = linearJacobian(:, 5);
    diagnostic.finite_difference = finiteDifference;
    diagnostic.tool_tangent_basis = toolTangentBasis;
    diagnostic.tool_direction_jacobian_3d = toolDirectionJacobian3;
    diagnostic.tool_direction_jacobian_2d = toolDirectionJacobian2;
    diagnostic.writing_task_jacobian = writingTaskJacobian;
    diagnostic.writing_task_jacobian_shape = size(writingTaskJacobian);
    diagnostic.writing_task_rank = rank(writingTaskJacobian);
    diagnostic.writing_task_rank_tolerance = defaultRankTolerance;
    diagnostic.writing_task_singular_values = singularValues;
    diagnostic.writing_task_condition_number = cond(writingTaskJacobian);
    diagnostic.j5_task_column = writingTaskJacobian(:, 5);
    diagnostic.j5_is_task_invisible_at_q_anchor = ...
        norm(diagnostic.j5_task_column) <= defaultRankTolerance;
end


function finiteDifference = j5_finite_difference(robot, qAnchor, endEffector, basePose)

    jointLimits = robot.Bodies{5}.Joint.PositionLimits;
    deltaRad = 1.0e-4;
    qPlus = qAnchor;
    qMinus = qAnchor;
    qPlus(5) = qPlus(5) + deltaRad;
    qMinus(5) = qMinus(5) - deltaRad;

    if qPlus(5) > jointLimits(2) || qMinus(5) < jointLimits(1)
        error( ...
            'MonoTeach:J5FiniteDifferenceOutsideLimits', ...
            'q_anchor +/- the J5 finite-difference delta must stay in limits.');
    end

    plusPose = getTransform(robot, qPlus, endEffector);
    minusPose = getTransform(robot, qMinus, endEffector);
    baseXYZ = basePose(1:3, 4);
    baseToolZ = basePose(1:3, 3);
    plusXYZ = plusPose(1:3, 4);
    minusXYZ = minusPose(1:3, 4);
    plusToolZ = plusPose(1:3, 3);
    minusToolZ = minusPose(1:3, 3);

    finiteDifference = struct();
    finiteDifference.delta_rad = deltaRad;
    finiteDifference.q_plus = qPlus;
    finiteDifference.q_minus = qMinus;
    finiteDifference.body5_origin_xyz_m = baseXYZ';
    finiteDifference.body5_origin_plus_xyz_m = plusXYZ';
    finiteDifference.body5_origin_minus_xyz_m = minusXYZ';
    finiteDifference.tool_z_base = baseToolZ';
    finiteDifference.tool_z_plus = plusToolZ';
    finiteDifference.tool_z_minus = minusToolZ';
    finiteDifference.position_delta_plus_m = (plusXYZ - baseXYZ)';
    finiteDifference.position_delta_minus_m = (minusXYZ - baseXYZ)';
    finiteDifference.position_delta_norm_plus_m = ...
        norm(plusXYZ - baseXYZ);
    finiteDifference.position_delta_norm_minus_m = ...
        norm(minusXYZ - baseXYZ);
    finiteDifference.central_position_derivative_m_per_rad = ...
        ((plusXYZ - minusXYZ) / (2 * deltaRad))';
    finiteDifference.tool_z_delta_plus = (plusToolZ - baseToolZ)';
    finiteDifference.tool_z_delta_minus = (minusToolZ - baseToolZ)';
    finiteDifference.tool_z_angular_delta_plus_rad = ...
        angle_between_unit_vectors(baseToolZ, plusToolZ);
    finiteDifference.tool_z_angular_delta_minus_rad = ...
        angle_between_unit_vectors(baseToolZ, minusToolZ);
    finiteDifference.central_tool_z_derivative_per_rad = ...
        ((plusToolZ - minusToolZ) / (2 * deltaRad))';
end


function validate_inputs(robot, writingPostureConfig)

    if ~isa(robot, 'rigidBodyTree') || ~strcmp(robot.DataFormat, 'row') || ...
            robot.NumBodies ~= 5
        error( ...
            'MonoTeach:InvalidWritingTaskDofRobot', ...
            'robot must be the row-format Legacy 5DOF rigidBodyTree.');
    end

    if ~isstruct(writingPostureConfig) || ...
            ~strcmp(string(writingPostureConfig.end_effector), "body5") || ...
            ~strcmp(string(writingPostureConfig.tool_axis), "z")
        error( ...
            'MonoTeach:InvalidWritingTaskDofToolFrame', ...
            'This diagnostic requires body5 local +Z as the writing axis.');
    end
end


function matrix = skew_symmetric(vector)

    matrix = [ ...
        0, -vector(3), vector(2); ...
        vector(3), 0, -vector(1); ...
        -vector(2), vector(1), 0];
end


function angleRad = angle_between_unit_vectors(first, second)

    angleRad = acos(max(-1.0, min(1.0, dot(first, second))));
end
