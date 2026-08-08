"""
test_stage0.py

MonoTeach Stage 0 自动化测试。

运行：

    pytest -q

测试内容：
1. RobotModel
2. Modified DH
3. FK
4. IK -> FK
5. 五次多项式
6. Cartesian trajectory
7. Cartesian -> Joint trajectory
"""

import numpy as np

from stage0.robot_model import (
    LEGACY_ROBOT,
)

from stage0.kinematics import (
    modified_dh_matrix,
    forward_kinematics,
    inverse_kinematics,
    pose_error,
)

from stage0.trajectory import (
    quintic_coefficients,
    evaluate_quintic,
    TrajectoryPlanner,
)


# ============================================================
# RobotModel
# ============================================================

def test_robot_model_basic():

    assert LEGACY_ROBOT.dof == 5

    assert LEGACY_ROBOT.dh_params.shape == (
        5,
        3,
    )

    assert (
        LEGACY_ROBOT
        .joint_limits_deg
        .shape
        == (5, 2)
    )

    assert (
        LEGACY_ROBOT
        .default_joint_angles_deg
        .shape
        == (5,)
    )


def test_default_joint_angles_valid():

    assert (
        LEGACY_ROBOT
        .are_joint_angles_valid(
            LEGACY_ROBOT
            .default_joint_angles_deg
        )
    )


def test_invalid_joint_angles():

    q = np.array(
        [
            200.0,
            90.0,
            0.0,
            90.0,
            0.0,
        ]
    )

    assert not (
        LEGACY_ROBOT
        .are_joint_angles_valid(
            q
        )
    )


# ============================================================
# Modified DH
# ============================================================

def test_modified_dh_matrix_shape():

    T = modified_dh_matrix(
        a=0.085,
        alpha_deg=180.0,
        d=0.0,
        theta_deg=30.0,
    )

    assert T.shape == (
        4,
        4,
    )

    assert np.allclose(
        T[3],
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )


def test_modified_dh_rotation_valid():

    T = modified_dh_matrix(
        a=0.08,
        alpha_deg=90.0,
        d=0.1,
        theta_deg=45.0,
    )

    R = T[:3, :3]

    # 旋转矩阵应满足：
    #
    # R^T R = I

    assert np.allclose(
        R.T @ R,
        np.eye(3),
        atol=1e-10,
    )

    # det(R) = 1

    assert np.isclose(
        np.linalg.det(R),
        1.0,
        atol=1e-10,
    )


# ============================================================
# FK
# ============================================================

def test_forward_kinematics():

    q = np.array(
        [
            0.0,
            90.0,
            0.0,
            90.0,
            0.0,
        ]
    )

    position, T = (
        forward_kinematics(
            q
        )
    )

    assert position.shape == (
        3,
    )

    assert T.shape == (
        4,
        4,
    )

    assert np.all(
        np.isfinite(
            position
        )
    )

    assert np.all(
        np.isfinite(
            T
        )
    )

    assert np.allclose(
        T[3],
        [
            0.0,
            0.0,
            0.0,
            1.0,
        ],
    )


# ============================================================
# IK
# ============================================================

def test_fk_ik_fk_consistency():

    q_original = np.array(
        [
            25.0,
            75.0,
            15.0,
            100.0,
            20.0,
        ]
    )

    _, T_target = (
        forward_kinematics(
            q_original
        )
    )

    seed = (
        q_original
        + np.array(
            [
                3.0,
                -3.0,
                2.0,
                2.0,
                -2.0,
            ]
        )
    )

    result = inverse_kinematics(
        T_target=T_target,
        seed_deg=seed,
    )

    assert result.success

    _, T_result = (
        forward_kinematics(
            result.joint_angles_deg
        )
    )

    (
        position_error,
        orientation_error,
    ) = pose_error(
        T_result,
        T_target,
    )

    assert (
        position_error
        < 1e-4
    )

    assert (
        orientation_error
        < 0.2
    )


def test_ik_solution_inside_limits():

    q_original = np.array(
        [
            -20.0,
            80.0,
            10.0,
            100.0,
            -30.0,
        ]
    )

    _, T_target = (
        forward_kinematics(
            q_original
        )
    )

    result = inverse_kinematics(
        T_target=T_target,
        seed_deg=q_original,
    )

    assert result.success

    assert (
        LEGACY_ROBOT
        .are_joint_angles_valid(
            result.joint_angles_deg
        )
    )


# ============================================================
# Quintic
# ============================================================

def test_quintic_boundary_conditions():

    coefficients = (
        quintic_coefficients(
            q0=0.2,
            qf=0.8,
            duration_s=3.0,
            v0=0.0,
            vf=0.0,
            acc0=0.0,
            accf=0.0,
        )
    )

    (
        p0,
        v0,
        a0,
    ) = evaluate_quintic(
        coefficients,
        0.0,
    )

    (
        pf,
        vf,
        af,
    ) = evaluate_quintic(
        coefficients,
        3.0,
    )

    assert np.isclose(
        p0,
        0.2,
        atol=1e-10,
    )

    assert np.isclose(
        pf,
        0.8,
        atol=1e-10,
    )

    assert np.isclose(
        v0,
        0.0,
        atol=1e-10,
    )

    assert np.isclose(
        vf,
        0.0,
        atol=1e-10,
    )

    assert np.isclose(
        a0,
        0.0,
        atol=1e-10,
    )

    assert np.isclose(
        af,
        0.0,
        atol=1e-10,
    )


# ============================================================
# Cartesian trajectory
# ============================================================

def test_cartesian_segment():

    planner = (
        TrajectoryPlanner()
    )

    start = np.array(
        [
            0.15,
            0.05,
            0.20,
            -30.0,
        ]
    )

    target = np.array(
        [
            0.18,
            0.08,
            0.22,
            -45.0,
        ]
    )

    trajectory = (
        planner.plan_segment(
            start_pose=start,
            target_pose=target,
            duration_s=2.0,
            sample_rate_hz=10.0,
        )
    )

    assert (
        trajectory.num_points
        == 21
    )

    assert np.allclose(
        trajectory.poses[0],
        start,
    )

    assert np.allclose(
        trajectory.poses[-1],
        target,
    )

    assert np.isclose(
        trajectory.duration_s,
        2.0,
    )


# ============================================================
# Cartesian -> Joint
# ============================================================

def test_cartesian_to_joint():

    planner = (
        TrajectoryPlanner()
    )

    # 先取两个已知可达关节姿态

    q_start = np.array(
        [
            10.0,
            80.0,
            10.0,
            100.0,
            0.0,
        ]
    )

    q_end = np.array(
        [
            20.0,
            75.0,
            15.0,
            100.0,
            0.0,
        ]
    )

    (
        xyz_start,
        T_start,
    ) = forward_kinematics(
        q_start
    )

    (
        xyz_end,
        T_end,
    ) = forward_kinematics(
        q_end
    )

    # 从 Tool Z轴提取旧项目定义的 pitch

    def get_pitch(T):

        z_axis = (
            T[:3, 2]
        )

        horizontal = np.sqrt(
            z_axis[0] ** 2
            + z_axis[1] ** 2
        )

        return np.rad2deg(
            np.arctan2(
                -z_axis[2],
                horizontal,
            )
        )

    start_pose = np.array(
        [
            *xyz_start,
            get_pitch(
                T_start
            ),
        ]
    )

    target_pose = np.array(
        [
            *xyz_end,
            get_pitch(
                T_end
            ),
        ]
    )

    cartesian = (
        planner.plan_segment(
            start_pose=start_pose,
            target_pose=target_pose,
            duration_s=1.0,
            sample_rate_hz=5.0,
        )
    )

    joint = (
        planner.cartesian_to_joint(
            cartesian_trajectory=cartesian,
            initial_joint_angles_deg=q_start,
        )
    )

    assert (
        joint.joint_angles_deg.shape
        ==
        (
            cartesian.num_points,
            5,
        )
    )

    assert (
        joint.max_position_error_mm
        < 0.2
    )

    assert (
        joint.max_orientation_error_deg
        < 0.5
    )

    for q in (
        joint.joint_angles_deg
    ):

        assert (
            LEGACY_ROBOT
            .are_joint_angles_valid(
                q
            )
        )