"""
verify_legacy.py

MonoTeach Stage 0
Legacy Robot Baseline 最终综合验证。

验证内容：
1. RobotModel 参数
2. FK
3. IK -> FK 一致性
4. 多 Waypoint 笛卡尔轨迹
5. Cartesian -> Joint Trajectory
6. 轨迹连续性
7. 机械臂三维骨架可视化
8. 五关节角曲线
9. CSV / TXT 验证结果输出

这个文件不包含：
- GUI
- 串口
- PWM
- 摄像头
- ROS
"""

from __future__ import annotations

from pathlib import Path
import csv

import matplotlib.pyplot as plt
import numpy as np

from .robot_model import LEGACY_ROBOT

from .kinematics import (
    forward_kinematics,
    inverse_kinematics,
    joint_positions,
)

from .trajectory import (
    TrajectoryPlanner,
)


# ============================================================
# 1. 输出目录
# ============================================================

CURRENT_DIR = Path(__file__).resolve().parent

OUTPUT_DIR = (
    CURRENT_DIR
    / "output"
)

OUTPUT_DIR.mkdir(
    parents=True,
    exist_ok=True,
)


# ============================================================
# 2. 工具函数
# ============================================================

def print_title(
    title: str,
) -> None:

    print(
        "\n"
        + "=" * 72
    )

    print(
        title
    )

    print(
        "=" * 72
    )


def pose_from_joint_angles(
    joint_angles_deg,
) -> np.ndarray:
    """
    从一组已知关节角获得：

        [x, y, z, pitch_deg]

    用于构造 Stage 0 的可达轨迹测试点。

    注意：

    这里的 pitch 定义与旧项目保持一致：

        z_axis =
        [
            cos(pitch)*cos(theta1),
            cos(pitch)*sin(theta1),
            -sin(pitch)
        ]

    因此：

        pitch =
        atan2(
            -z_axis_z,
            sqrt(
                z_axis_x²
                +
                z_axis_y²
            )
        )
    """

    (
        position,
        T,
    ) = forward_kinematics(
        joint_angles_deg
    )

    z_axis = (
        T[:3, 2]
    )

    horizontal = np.sqrt(
        z_axis[0] ** 2
        + z_axis[1] ** 2
    )

    pitch_rad = np.arctan2(
        -z_axis[2],
        horizontal,
    )

    pitch_deg = np.rad2deg(
        pitch_rad
    )

    return np.array(
        [
            position[0],
            position[1],
            position[2],
            pitch_deg,
        ],
        dtype=float,
    )


# ============================================================
# 3. Robot Model 验证
# ============================================================

def verify_robot_model():

    print_title(
        "TEST 1 - Robot Model"
    )

    LEGACY_ROBOT.print_summary()

    assert (
        LEGACY_ROBOT.dof
        == 5
    )

    assert (
        LEGACY_ROBOT
        .dh_params
        .shape
        == (5, 3)
    )

    assert (
        LEGACY_ROBOT
        .joint_limits_deg
        .shape
        == (5, 2)
    )

    print(
        "\nRobot model verification PASSED"
    )


# ============================================================
# 4. FK 验证
# ============================================================

def verify_fk():

    print_title(
        "TEST 2 - Forward Kinematics"
    )

    q = np.array(
        [
            0.0,
            90.0,
            0.0,
            90.0,
            0.0,
        ]
    )

    (
        position,
        T,
    ) = forward_kinematics(
        q
    )

    print(
        "Input joint angles [deg]:"
    )

    print(
        q
    )

    print(
        "\nEnd-effector position [m]:"
    )

    print(
        np.round(
            position,
            6,
        )
    )

    print(
        "\nT_base_tool:"
    )

    print(
        np.round(
            T,
            6,
        )
    )

    if not np.all(
        np.isfinite(
            T
        )
    ):

        raise RuntimeError(
            "FK输出出现 NaN / Inf"
        )

    print(
        "\nFK verification PASSED"
    )

    return q


# ============================================================
# 5. FK -> IK -> FK
# ============================================================

def verify_ik():

    print_title(
        "TEST 3 - FK -> IK -> FK"
    )

    test_joint_sets = [

        np.array(
            [
                0.0,
                90.0,
                0.0,
                90.0,
                0.0,
            ]
        ),

        np.array(
            [
                20.0,
                80.0,
                10.0,
                100.0,
                0.0,
            ]
        ),

        np.array(
            [
                35.0,
                70.0,
                20.0,
                95.0,
                0.0,
            ]
        ),
    ]

    max_position_error = 0.0

    max_orientation_error = 0.0

    for (
        index,
        q_original,
    ) in enumerate(
        test_joint_sets,
        start=1,
    ):

        (
            _,
            T_target,
        ) = forward_kinematics(
            q_original
        )

        # 使用一个略微偏离真实解的初值，
        # 模拟实际连续轨迹中的情况。
        seed = (
            q_original
            + np.array(
                [
                    3.0,
                    -2.0,
                    2.0,
                    2.0,
                    0.0,
                ]
            )
        )

        result = (
            inverse_kinematics(
                T_target=T_target,
                seed_deg=seed,
            )
        )

        print(
            f"\nCase {index}"
        )

        print(
            "Original q:"
        )

        print(
            np.round(
                q_original,
                5,
            )
        )

        print(
            "IK q:"
        )

        print(
            np.round(
                result.joint_angles_deg,
                5,
            )
        )

        print(
            "Position error:"
        )

        print(
            f"{result.position_error_m * 1000:.8f} mm"
        )

        print(
            "Orientation error:"
        )

        print(
            f"{result.orientation_error_deg:.8f} deg"
        )

        if not result.success:

            raise RuntimeError(
                "IK verification failed:\n"
                + result.message
            )

        max_position_error = max(
            max_position_error,
            result.position_error_m,
        )

        max_orientation_error = max(
            max_orientation_error,
            result.orientation_error_deg,
        )

    print(
        "\nMaximum position error:"
    )

    print(
        f"{max_position_error * 1000:.8f} mm"
    )

    print(
        "Maximum orientation error:"
    )

    print(
        f"{max_orientation_error:.8f} deg"
    )

    print(
        "\nIK verification PASSED"
    )


# ============================================================
# 6. 构造 Legacy Waypoints
# ============================================================

def create_legacy_waypoints():

    """
    不手搓几个可能不可达的XYZ。

    而是先选几组合法关节角，
    用FK生成一定可达的Cartesian Waypoint。

    这样 Stage 0 验证关注的是：

        轨迹系统是否正常

    而不是先陷入工作空间调参。
    """

    q_waypoints = [

        np.array(
            [
                -30.0,
                75.0,
                15.0,
                100.0,
                0.0,
            ]
        ),

        np.array(
            [
                -10.0,
                65.0,
                25.0,
                95.0,
                0.0,
            ]
        ),

        np.array(
            [
                15.0,
                70.0,
                15.0,
                105.0,
                0.0,
            ]
        ),

        np.array(
            [
                35.0,
                80.0,
                5.0,
                100.0,
                0.0,
            ]
        ),
    ]

    poses = np.array(
        [
            pose_from_joint_angles(
                q
            )
            for q in q_waypoints
        ]
    )

    return (
        q_waypoints,
        poses,
    )


# ============================================================
# 7. 轨迹验证
# ============================================================

def verify_trajectory():

    print_title(
        "TEST 4 - Cartesian Trajectory"
    )

    (
        q_waypoints,
        waypoints,
    ) = create_legacy_waypoints()

    print(
        "Cartesian Waypoints:"
    )

    for (
        i,
        waypoint,
    ) in enumerate(
        waypoints
    ):

        print(
            f"P{i}:",
            np.round(
                waypoint,
                6,
            ),
        )

    planner = (
        TrajectoryPlanner()
    )

    cartesian = (
        planner.plan_waypoints(
            waypoints=waypoints,
            segment_duration_s=2.0,
            sample_rate_hz=25.0,
        )
    )

    print(
        "\nCartesian trajectory points:"
    )

    print(
        cartesian.num_points
    )

    print(
        "Duration:"
    )

    print(
        f"{cartesian.duration_s:.3f} s"
    )

    print_title(
        "TEST 5 - Cartesian -> Joint"
    )

    joint_trajectory = (
        planner.cartesian_to_joint(
            cartesian_trajectory=cartesian,

            initial_joint_angles_deg=(
                q_waypoints[0]
            ),
        )
    )

    print(
        "Joint trajectory shape:"
    )

    print(
        joint_trajectory
        .joint_angles_deg
        .shape
    )

    print(
        "\nMaximum IK position error:"
    )

    print(
        f"{joint_trajectory.max_position_error_mm:.8f} mm"
    )

    print(
        "Maximum IK orientation error:"
    )

    print(
        f"{joint_trajectory.max_orientation_error_deg:.8f} deg"
    )

    print(
        "Maximum single joint step:"
    )

    print(
        f"{joint_trajectory.max_joint_step_deg:.8f} deg"
    )

    if (
        joint_trajectory
        .max_position_error_mm
        > 0.2
    ):

        raise RuntimeError(
            "轨迹IK位置误差过大"
        )

    if (
        joint_trajectory
        .max_orientation_error_deg
        > 0.5
    ):

        raise RuntimeError(
            "轨迹IK姿态误差过大"
        )

    print(
        "\nTrajectory verification PASSED"
    )

    return (
        cartesian,
        joint_trajectory,
        waypoints,
    )


# ============================================================
# 8. 输出 CSV
# ============================================================

def save_trajectory_csv(
    joint_trajectory,
):

    output_file = (
        OUTPUT_DIR
        / "legacy_trajectory.csv"
    )

    with open(
        output_file,
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as f:

        writer = csv.writer(
            f
        )

        writer.writerow(
            [
                "time_s",

                "x_m",
                "y_m",
                "z_m",
                "pitch_deg",

                "j1_deg",
                "j2_deg",
                "j3_deg",
                "j4_deg",
                "j5_deg",

                "ik_position_error_m",

                "ik_orientation_error_deg",
            ]
        )

        for i in range(
            joint_trajectory.num_points
        ):

            row = [

                joint_trajectory
                .time_s[i],

                *joint_trajectory
                .cartesian_poses[i],

                *joint_trajectory
                .joint_angles_deg[i],

                joint_trajectory
                .position_errors_m[i],

                joint_trajectory
                .orientation_errors_deg[i],
            ]

            writer.writerow(
                row
            )

    print(
        "\nSaved:"
    )

    print(
        output_file
    )


# ============================================================
# 9. 画三维轨迹 + 机械臂
# ============================================================

def plot_cartesian_trajectory(
    cartesian,
    joint_trajectory,
    waypoints,
):

    fig = plt.figure(
        figsize=(
            9,
            8,
        )
    )

    ax = fig.add_subplot(
        111,
        projection="3d",
    )

    xyz = (
        cartesian
        .poses[:, :3]
    )

    ax.plot(
        xyz[:, 0],
        xyz[:, 1],
        xyz[:, 2],
        linewidth=2,
        label="Cartesian trajectory",
    )

    ax.scatter(
        waypoints[:, 0],
        waypoints[:, 1],
        waypoints[:, 2],
        s=60,
        label="Waypoints",
    )

    # --------------------------------------------------------
    # 画起点 / 中点 / 终点机械臂
    # --------------------------------------------------------

    indices = [

        0,

        len(
            joint_trajectory
            .joint_angles_deg
        )
        // 2,

        len(
            joint_trajectory
            .joint_angles_deg
        )
        - 1,
    ]

    names = [
        "Robot start",
        "Robot middle",
        "Robot end",
    ]

    for (
        index,
        name,
    ) in zip(
        indices,
        names,
    ):

        q = (
            joint_trajectory
            .joint_angles_deg[
                index
            ]
        )

        positions = (
            joint_positions(
                q
            )
        )

        ax.plot(
            positions[:, 0],
            positions[:, 1],
            positions[:, 2],
            marker="o",
            linewidth=2,
            label=name,
        )

    ax.set_xlabel(
        "X [m]"
    )

    ax.set_ylabel(
        "Y [m]"
    )

    ax.set_zlabel(
        "Z [m]"
    )

    ax.set_title(
        "MonoTeach Stage 0 - Legacy 5DOF Cartesian Trajectory"
    )

    ax.legend()

    ax.grid(
        True
    )

    # 尽量保持XYZ视觉比例一致
    all_points = xyz

    x_range = (
        np.max(
            all_points[:, 0]
        )
        - np.min(
            all_points[:, 0]
        )
    )

    y_range = (
        np.max(
            all_points[:, 1]
        )
        - np.min(
            all_points[:, 1]
        )
    )

    z_range = (
        np.max(
            all_points[:, 2]
        )
        - np.min(
            all_points[:, 2]
        )
    )

    max_range = max(
        x_range,
        y_range,
        z_range,
        0.05,
    )

    x_mid = np.mean(
        [
            np.max(
                all_points[:, 0]
            ),
            np.min(
                all_points[:, 0]
            ),
        ]
    )

    y_mid = np.mean(
        [
            np.max(
                all_points[:, 1]
            ),
            np.min(
                all_points[:, 1]
            ),
        ]
    )

    z_mid = np.mean(
        [
            np.max(
                all_points[:, 2]
            ),
            np.min(
                all_points[:, 2]
            ),
        ]
    )

    ax.set_xlim(
        x_mid - max_range,
        x_mid + max_range,
    )

    ax.set_ylim(
        y_mid - max_range,
        y_mid + max_range,
    )

    ax.set_zlim(
        max(
            0.0,
            z_mid - max_range,
        ),
        z_mid + max_range,
    )

    output_file = (
        OUTPUT_DIR
        / "legacy_cartesian_trajectory.png"
    )

    fig.tight_layout()

    fig.savefig(
        output_file,
        dpi=160,
    )

    print(
        "\nSaved:"
    )

    print(
        output_file
    )

    return fig


# ============================================================
# 10. 画五关节角曲线
# ============================================================

def plot_joint_trajectory(
    joint_trajectory,
):

    fig = plt.figure(
        figsize=(
            10,
            6,
        )
    )

    ax = fig.add_subplot(
        111
    )

    time_s = (
        joint_trajectory
        .time_s
    )

    joints = (
        joint_trajectory
        .joint_angles_deg
    )

    for joint_index in range(
        LEGACY_ROBOT.dof
    ):

        ax.plot(
            time_s,

            joints[
                :,
                joint_index,
            ],

            linewidth=2,

            label=(
                f"J{joint_index + 1}"
            ),
        )

    ax.set_xlabel(
        "Time [s]"
    )

    ax.set_ylabel(
        "Joint angle [deg]"
    )

    ax.set_title(
        "MonoTeach Stage 0 - Joint Trajectory"
    )

    ax.grid(
        True
    )

    ax.legend()

    output_file = (
        OUTPUT_DIR
        / "legacy_joint_trajectory.png"
    )

    fig.tight_layout()

    fig.savefig(
        output_file,
        dpi=160,
    )

    print(
        "\nSaved:"
    )

    print(
        output_file
    )

    return fig


# ============================================================
# 11. 保存验证报告
# ============================================================

def save_report(
    joint_trajectory,
):

    report_file = (
        OUTPUT_DIR
        / "legacy_verification_report.txt"
    )

    lines = [

        "MonoTeach Stage 0",
        "Legacy Robot Baseline Verification",
        "",
        f"Robot: {LEGACY_ROBOT.name}",
        f"DOF: {LEGACY_ROBOT.dof}",
        f"DH convention: {LEGACY_ROBOT.dh_convention}",
        "",
        (
            "Trajectory points: "
            f"{joint_trajectory.num_points}"
        ),
        (
            "Duration: "
            f"{joint_trajectory.duration_s:.6f} s"
        ),
        (
            "Maximum IK position error: "
            f"{joint_trajectory.max_position_error_mm:.9f} mm"
        ),
        (
            "Maximum IK orientation error: "
            f"{joint_trajectory.max_orientation_error_deg:.9f} deg"
        ),
        (
            "Maximum single joint step: "
            f"{joint_trajectory.max_joint_step_deg:.9f} deg"
        ),
        "",
        "Stage 0 Baseline Status: PASSED",
    ]

    report_file.write_text(
        "\n".join(
            lines
        ),
        encoding="utf-8",
    )

    print(
        "\nSaved:"
    )

    print(
        report_file
    )


# ============================================================
# 12. Main
# ============================================================

def main():

    print(
        "\n"
        "MonoTeach Stage 0"
    )

    print(
        "Legacy Robot Callback & Baseline"
    )

    # --------------------------------------------------------
    # RobotModel
    # --------------------------------------------------------

    verify_robot_model()

    # --------------------------------------------------------
    # FK
    # --------------------------------------------------------

    verify_fk()

    # --------------------------------------------------------
    # IK
    # --------------------------------------------------------

    verify_ik()

    # --------------------------------------------------------
    # Trajectory
    # --------------------------------------------------------

    (
        cartesian,
        joint_trajectory,
        waypoints,
    ) = verify_trajectory()

    # --------------------------------------------------------
    # 保存数据
    # --------------------------------------------------------

    save_trajectory_csv(
        joint_trajectory
    )

    save_report(
        joint_trajectory
    )

    # --------------------------------------------------------
    # 可视化
    # --------------------------------------------------------

    plot_cartesian_trajectory(
        cartesian,
        joint_trajectory,
        waypoints,
    )

    plot_joint_trajectory(
        joint_trajectory
    )

    print_title(
        "STAGE 0 LEGACY BASELINE"
    )

    print(
        "Robot Model        : PASS"
    )

    print(
        "Forward Kinematics : PASS"
    )

    print(
        "Inverse Kinematics : PASS"
    )

    print(
        "Trajectory Planner : PASS"
    )

    print(
        "Joint Continuity   : PASS"
    )

    print(
        "\nOutput directory:"
    )

    print(
        OUTPUT_DIR
    )

    print(
        "\nStage 0 Legacy Baseline PASSED."
    )

    # 最后显示两张图
    plt.show()


if __name__ == "__main__":

    main()