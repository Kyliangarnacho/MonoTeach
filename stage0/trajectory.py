"""
trajectory.py

MonoTeach Stage 0
5DOF 机械臂轨迹规划核心。

职责：
1. 五次多项式插值
2. 笛卡尔空间 [x, y, z, pitch] 轨迹生成
3. 多 Waypoint 轨迹连接
4. Cartesian Trajectory -> Joint Trajectory
5. 使用“上一点 IK 解作为下一点 seed”保证轨迹连续性

注意：
Stage 0 暂时采用：

    每个 waypoint 处速度 = 0
    每个 waypoint 处加速度 = 0

因此经过 waypoint 时机械臂会理论上短暂停稳。

这符合旧项目 Legacy Baseline 的思路。

后续视觉示教阶段会升级为：
    连续轨迹平滑
    重采样
    滤波
    速度/加速度限制
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from .robot_model import (
    LEGACY_ROBOT,
    RobotModel,
)

from .kinematics import (
    forward_kinematics,
    inverse_kinematics,
    target_transform_from_xyz_pitch,
)


# ============================================================
# 1. 轨迹数据结构
# ============================================================

@dataclass
class CartesianTrajectory:
    """
    笛卡尔空间轨迹。

    poses:
        shape = (N, 4)

        每一行为：

        [x, y, z, pitch_deg]

    time_s:
        shape = (N,)
    """

    time_s: np.ndarray
    poses: np.ndarray

    def __post_init__(self):

        self.time_s = np.asarray(
            self.time_s,
            dtype=float,
        )

        self.poses = np.asarray(
            self.poses,
            dtype=float,
        )

        if self.time_s.ndim != 1:

            raise ValueError(
                "time_s 必须是一维数组"
            )

        if (
            self.poses.ndim != 2
            or self.poses.shape[1] != 4
        ):

            raise ValueError(
                "poses 必须为 shape=(N, 4)"
            )

        if len(self.time_s) != len(
            self.poses
        ):

            raise ValueError(
                "time_s 和 poses 长度必须一致"
            )

        if len(self.time_s) < 2:

            raise ValueError(
                "轨迹至少需要两个采样点"
            )

        if np.any(
            np.diff(self.time_s) <= 0
        ):

            raise ValueError(
                "time_s 必须严格递增"
            )

    @property
    def duration_s(self) -> float:

        return float(
            self.time_s[-1]
            - self.time_s[0]
        )

    @property
    def num_points(self) -> int:

        return int(
            len(self.time_s)
        )


@dataclass
class JointTrajectory:
    """
    关节空间轨迹。

    joint_angles_deg:
        shape = (N, 5)

    cartesian_poses:
        shape = (N, 4)

    position_errors_m:
        IK 后 FK 的位置误差

    orientation_errors_deg:
        IK 后 FK 的姿态误差
    """

    time_s: np.ndarray

    cartesian_poses: np.ndarray

    joint_angles_deg: np.ndarray

    position_errors_m: np.ndarray

    orientation_errors_deg: np.ndarray

    def __post_init__(self):

        self.time_s = np.asarray(
            self.time_s,
            dtype=float,
        )

        self.cartesian_poses = np.asarray(
            self.cartesian_poses,
            dtype=float,
        )

        self.joint_angles_deg = np.asarray(
            self.joint_angles_deg,
            dtype=float,
        )

        self.position_errors_m = np.asarray(
            self.position_errors_m,
            dtype=float,
        )

        self.orientation_errors_deg = np.asarray(
            self.orientation_errors_deg,
            dtype=float,
        )

        n = len(
            self.time_s
        )

        if self.cartesian_poses.shape != (
            n,
            4,
        ):

            raise ValueError(
                "cartesian_poses 尺寸错误"
            )

        if (
            self.joint_angles_deg.ndim
            != 2
        ):

            raise ValueError(
                "joint_angles_deg "
                "必须为二维数组"
            )

        if (
            len(self.joint_angles_deg)
            != n
        ):

            raise ValueError(
                "joint_angles_deg "
                "长度错误"
            )

        if self.position_errors_m.shape != (
            n,
        ):

            raise ValueError(
                "position_errors_m "
                "长度错误"
            )

        if (
            self.orientation_errors_deg.shape
            != (n,)
        ):

            raise ValueError(
                "orientation_errors_deg "
                "长度错误"
            )

    @property
    def duration_s(self) -> float:

        return float(
            self.time_s[-1]
            - self.time_s[0]
        )

    @property
    def num_points(self) -> int:

        return int(
            len(self.time_s)
        )

    @property
    def max_position_error_mm(
        self
    ) -> float:

        return float(
            np.max(
                self.position_errors_m
            )
            * 1000.0
        )

    @property
    def max_orientation_error_deg(
        self
    ) -> float:

        return float(
            np.max(
                self.orientation_errors_deg
            )
        )

    @property
    def max_joint_step_deg(
        self
    ) -> float:

        if len(
            self.joint_angles_deg
        ) < 2:

            return 0.0

        delta = np.diff(
            self.joint_angles_deg,
            axis=0,
        )

        return float(
            np.max(
                np.abs(
                    delta
                )
            )
        )


# ============================================================
# 2. 五次多项式
# ============================================================

def quintic_coefficients(
    q0: float,
    qf: float,
    duration_s: float,
    v0: float = 0.0,
    vf: float = 0.0,
    acc0: float = 0.0,
    accf: float = 0.0,
) -> np.ndarray:
    """
    求五次多项式系数：

        q(t)
        =
        c0
        + c1*t
        + c2*t²
        + c3*t³
        + c4*t⁴
        + c5*t⁵

    满足：

        q(0) = q0
        q(T) = qf

        q'(0) = v0
        q'(T) = vf

        q''(0) = acc0
        q''(T) = accf

    与旧代码不同：
    这里不会出现 a0 参数被位置 q0 覆盖的问题。
    """

    T = float(
        duration_s
    )

    if T <= 0:

        raise ValueError(
            "duration_s 必须大于 0"
        )

    c0 = float(
        q0
    )

    c1 = float(
        v0
    )

    c2 = float(
        acc0
    ) / 2.0

    A = np.array(
        [
            [
                T ** 3,
                T ** 4,
                T ** 5,
            ],
            [
                3 * T ** 2,
                4 * T ** 3,
                5 * T ** 4,
            ],
            [
                6 * T,
                12 * T ** 2,
                20 * T ** 3,
            ],
        ],
        dtype=float,
    )

    b = np.array(
        [
            qf
            - (
                c0
                + c1 * T
                + c2 * T ** 2
            ),

            vf
            - (
                c1
                + 2 * c2 * T
            ),

            accf
            - 2 * c2,
        ],
        dtype=float,
    )

    c3, c4, c5 = np.linalg.solve(
        A,
        b,
    )

    return np.array(
        [
            c0,
            c1,
            c2,
            c3,
            c4,
            c5,
        ],
        dtype=float,
    )


def evaluate_quintic(
    coefficients: Iterable[float],
    t: float,
) -> tuple[
    float,
    float,
    float,
]:
    """
    计算五次多项式在时刻 t 的：

        position
        velocity
        acceleration
    """

    c = np.asarray(
        coefficients,
        dtype=float,
    )

    if c.shape != (
        6,
    ):

        raise ValueError(
            "五次多项式必须包含6个系数"
        )

    (
        c0,
        c1,
        c2,
        c3,
        c4,
        c5,
    ) = c

    t = float(
        t
    )

    position = (
        c0
        + c1 * t
        + c2 * t ** 2
        + c3 * t ** 3
        + c4 * t ** 4
        + c5 * t ** 5
    )

    velocity = (
        c1
        + 2 * c2 * t
        + 3 * c3 * t ** 2
        + 4 * c4 * t ** 3
        + 5 * c5 * t ** 4
    )

    acceleration = (
        2 * c2
        + 6 * c3 * t
        + 12 * c4 * t ** 2
        + 20 * c5 * t ** 3
    )

    return (
        float(position),
        float(velocity),
        float(acceleration),
    )


# ============================================================
# 3. TrajectoryPlanner
# ============================================================

class TrajectoryPlanner:
    """
    MonoTeach Stage 0 轨迹规划器。
    """

    def __init__(
        self,
        robot: RobotModel = LEGACY_ROBOT,
    ):

        self.robot = robot

    # ========================================================
    # 单段轨迹
    # ========================================================

    def plan_segment(
        self,
        start_pose: Iterable[float],
        target_pose: Iterable[float],
        duration_s: float = 2.0,
        sample_rate_hz: float = 50.0,
    ) -> CartesianTrajectory:
        """
        在两个：

            [x, y, z, pitch]

        位姿之间进行五次插值。

        每一个分量独立进行五次插值。
        """

        start_pose = np.asarray(
            start_pose,
            dtype=float,
        )

        target_pose = np.asarray(
            target_pose,
            dtype=float,
        )

        if start_pose.shape != (
            4,
        ):

            raise ValueError(
                "start_pose 必须是 "
                "[x, y, z, pitch]"
            )

        if target_pose.shape != (
            4,
        ):

            raise ValueError(
                "target_pose 必须是 "
                "[x, y, z, pitch]"
            )

        if duration_s <= 0:

            raise ValueError(
                "duration_s 必须 > 0"
            )

        if sample_rate_hz <= 0:

            raise ValueError(
                "sample_rate_hz 必须 > 0"
            )

        number_of_samples = max(
            2,
            int(
                round(
                    duration_s
                    * sample_rate_hz
                )
            )
            + 1,
        )

        time_s = np.linspace(
            0.0,
            duration_s,
            number_of_samples,
        )

        trajectory = np.zeros(
            (
                number_of_samples,
                4,
            ),
            dtype=float,
        )

        for dimension in range(
            4
        ):

            coefficients = (
                quintic_coefficients(
                    q0=start_pose[
                        dimension
                    ],
                    qf=target_pose[
                        dimension
                    ],
                    duration_s=duration_s,
                    v0=0.0,
                    vf=0.0,
                    acc0=0.0,
                    accf=0.0,
                )
            )

            for (
                sample_index,
                t,
            ) in enumerate(
                time_s
            ):

                (
                    position,
                    _,
                    _,
                ) = evaluate_quintic(
                    coefficients,
                    t,
                )

                trajectory[
                    sample_index,
                    dimension,
                ] = position

        return CartesianTrajectory(
            time_s=time_s,
            poses=trajectory,
        )

    # ========================================================
    # 多Waypoints
    # ========================================================

    def plan_waypoints(
        self,
        waypoints: Sequence[
            Iterable[float]
        ],
        segment_duration_s: (
            float
            | Sequence[float]
        ) = 2.0,
        sample_rate_hz: float = 50.0,
    ) -> CartesianTrajectory:
        """
        多路径点轨迹规划。

        waypoints：

            [
                [x0, y0, z0, pitch0],
                [x1, y1, z1, pitch1],
                [x2, y2, z2, pitch2],
                ...
            ]

        相邻 waypoint 之间分别使用五次插值。

        Stage 0：
        每个 waypoint 默认速度和加速度均为0。
        """

        waypoints_array = np.asarray(
            waypoints,
            dtype=float,
        )

        if (
            waypoints_array.ndim != 2
            or waypoints_array.shape[1]
            != 4
        ):

            raise ValueError(
                "waypoints 必须为 "
                "shape=(N, 4)"
            )

        number_of_waypoints = (
            len(
                waypoints_array
            )
        )

        if number_of_waypoints < 2:

            raise ValueError(
                "至少需要两个 waypoint"
            )

        number_of_segments = (
            number_of_waypoints
            - 1
        )

        # ----------------------------------------------------
        # 处理每段时间
        # ----------------------------------------------------

        if np.isscalar(
            segment_duration_s
        ):

            duration = float(
                segment_duration_s
            )

            if duration <= 0:

                raise ValueError(
                    "segment_duration_s "
                    "必须 > 0"
                )

            durations = np.full(
                number_of_segments,
                duration,
                dtype=float,
            )

        else:

            durations = np.asarray(
                segment_duration_s,
                dtype=float,
            )

            if durations.shape != (
                number_of_segments,
            ):

                raise ValueError(
                    "segment_duration_s "
                    "数量必须等于 waypoint数量-1"
                )

            if np.any(
                durations <= 0
            ):

                raise ValueError(
                    "所有段时间必须 > 0"
                )

        # ----------------------------------------------------
        # 逐段生成
        # ----------------------------------------------------

        all_time = []

        all_poses = []

        accumulated_time = 0.0

        for segment_index in range(
            number_of_segments
        ):

            segment = self.plan_segment(
                start_pose=(
                    waypoints_array[
                        segment_index
                    ]
                ),

                target_pose=(
                    waypoints_array[
                        segment_index + 1
                    ]
                ),

                duration_s=float(
                    durations[
                        segment_index
                    ]
                ),

                sample_rate_hz=(
                    sample_rate_hz
                ),
            )

            segment_time = (
                segment.time_s
                + accumulated_time
            )

            segment_poses = (
                segment.poses
            )

            # 第二段开始：
            #
            # 删除每段的第一个点，
            # 避免 waypoint 被重复保存两次。
            if segment_index > 0:

                segment_time = (
                    segment_time[1:]
                )

                segment_poses = (
                    segment_poses[1:]
                )

            all_time.append(
                segment_time
            )

            all_poses.append(
                segment_poses
            )

            accumulated_time += float(
                durations[
                    segment_index
                ]
            )

        time_s = np.concatenate(
            all_time
        )

        poses = np.vstack(
            all_poses
        )

        return CartesianTrajectory(
            time_s=time_s,
            poses=poses,
        )

    # ========================================================
    # Cartesian -> Joint
    # ========================================================

    def cartesian_to_joint(
        self,
        cartesian_trajectory: CartesianTrajectory,
        initial_joint_angles_deg: (
            Iterable[float]
            | None
        ) = None,
        max_allowed_joint_step_deg: (
            float
            | None
        ) = None,
    ) -> JointTrajectory:
        """
        将整条笛卡尔轨迹转换为关节轨迹。

        核心：

            第 i 个点的 IK 解

                    ↓

            作为第 i+1 个点的 seed

        这是旧项目原本“想做但没有真正完成”的
        连续IK思路。
        """

        if initial_joint_angles_deg is None:

            seed = (
                self.robot
                .default_joint_angles_deg
                .copy()
            )

        else:

            seed = np.asarray(
                initial_joint_angles_deg,
                dtype=float,
            )

            self.robot.assert_joint_angles_valid(
                seed
            )

        joint_trajectory = []

        position_errors = []

        orientation_errors = []

        previous_q = (
            seed.copy()
        )

        for (
            point_index,
            pose,
        ) in enumerate(
            cartesian_trajectory.poses
        ):

            (
                x,
                y,
                z,
                pitch_deg,
            ) = pose

            T_target = (
                target_transform_from_xyz_pitch(
                    x=x,
                    y=y,
                    z=z,
                    pitch_deg=pitch_deg,
                )
            )

            result = inverse_kinematics(
                T_target=T_target,
                seed_deg=previous_q,
                robot=self.robot,
            )

            if not result.success:

                raise RuntimeError(
                    "\n"
                    f"IK失败：路径点 "
                    f"{point_index}\n"
                    f"Pose = "
                    f"{np.round(pose, 6)}\n"
                    f"{result.message}"
                )

            q = (
                result
                .joint_angles_deg
                .copy()
            )

            self.robot.assert_joint_angles_valid(
                q
            )

            # ------------------------------------------------
            # 检查关节跳变
            # ------------------------------------------------

            if (
                max_allowed_joint_step_deg
                is not None
                and len(
                    joint_trajectory
                ) > 0
            ):

                max_step = float(
                    np.max(
                        np.abs(
                            q
                            - previous_q
                        )
                    )
                )

                if (
                    max_step
                    >
                    max_allowed_joint_step_deg
                ):

                    raise RuntimeError(
                        "\n"
                        "检测到关节角突变：\n"
                        f"路径点："
                        f"{point_index}\n"
                        f"最大单步变化："
                        f"{max_step:.3f}°\n"
                        f"允许值："
                        f"{max_allowed_joint_step_deg:.3f}°"
                    )

            joint_trajectory.append(
                q
            )

            position_errors.append(
                result.position_error_m
            )

            orientation_errors.append(
                result.orientation_error_deg
            )

            # -----------------------------------------------
            # 下一点使用当前解作为seed
            # -----------------------------------------------

            previous_q = (
                q
            )

        return JointTrajectory(
            time_s=(
                cartesian_trajectory
                .time_s
                .copy()
            ),

            cartesian_poses=(
                cartesian_trajectory
                .poses
                .copy()
            ),

            joint_angles_deg=np.asarray(
                joint_trajectory,
                dtype=float,
            ),

            position_errors_m=np.asarray(
                position_errors,
                dtype=float,
            ),

            orientation_errors_deg=np.asarray(
                orientation_errors,
                dtype=float,
            ),
        )


# ============================================================
# 4. Legacy兼容辅助接口
# ============================================================

def five_order_polynomial(
    t: float,
    t_total: float,
    q0: float,
    qf: float,
    v0: float = 0.0,
    vf: float = 0.0,
    a0: float = 0.0,
    af: float = 0.0,
) -> tuple[
    float,
    float,
    float,
]:
    """
    保留旧 final.py 函数风格的接口。

    但是内部已经修复旧代码的 a0 覆盖 bug。
    """

    coefficients = (
        quintic_coefficients(
            q0=q0,
            qf=qf,
            duration_s=t_total,
            v0=v0,
            vf=vf,
            acc0=a0,
            accf=af,
        )
    )

    return evaluate_quintic(
        coefficients,
        t,
    )


def check_joint_limits(
    joint_angles_deg: Iterable[float],
    robot: RobotModel = LEGACY_ROBOT,
) -> bool:
    """
    Legacy辅助接口。
    """

    return (
        robot
        .are_joint_angles_valid(
            joint_angles_deg
        )
    )


# ============================================================
# 5. 单文件测试
# ============================================================

if __name__ == "__main__":

    print(
        "=" * 68
    )

    print(
        "MonoTeach Stage 0 "
        "- trajectory.py self test"
    )

    print(
        "=" * 68
    )

    planner = (
        TrajectoryPlanner()
    )

    # ========================================================
    # TEST 1
    # 五次多项式边界条件
    # ========================================================

    print(
        "\n[TEST 1] "
        "Quintic boundary conditions"
    )

    coefficients = (
        quintic_coefficients(
            q0=0.0,
            qf=1.0,
            duration_s=2.0,
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
        2.0,
    )

    print(
        "Start:"
    )

    print(
        f"position     = {p0:.10f}"
    )

    print(
        f"velocity     = {v0:.10f}"
    )

    print(
        f"acceleration = {a0:.10f}"
    )

    print(
        "\nEnd:"
    )

    print(
        f"position     = {pf:.10f}"
    )

    print(
        f"velocity     = {vf:.10f}"
    )

    print(
        f"acceleration = {af:.10f}"
    )

    if not np.allclose(
        [
            p0,
            v0,
            a0,
            pf,
            vf,
            af,
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
        ],
        atol=1e-8,
    ):

        raise RuntimeError(
            "五次多项式边界条件测试失败"
        )

    # ========================================================
    # TEST 2
    # 单段笛卡尔轨迹
    # ========================================================

    print(
        "\n[TEST 2] "
        "Cartesian trajectory"
    )

    # 两个位置来自本机械臂已知可达姿态，
    # 因此适合用作 Stage 0 验证。
    start_pose = np.array(
        [
            0.16862920,
            0.09735811,
            0.24553517,
            -30.0,
        ],
        dtype=float,
    )

    target_pose = np.array(
        [
            0.12832857,
            0.04670778,
            0.29759057,
            -50.0,
        ],
        dtype=float,
    )

    cartesian = (
        planner.plan_segment(
            start_pose=start_pose,
            target_pose=target_pose,
            duration_s=2.0,
            sample_rate_hz=25.0,
        )
    )

    print(
        "Number of points:",
        cartesian.num_points,
    )

    print(
        "Duration:",
        f"{cartesian.duration_s:.3f} s",
    )

    print(
        "Start pose:",
        np.round(
            cartesian.poses[0],
            6,
        ),
    )

    print(
        "End pose:",
        np.round(
            cartesian.poses[-1],
            6,
        ),
    )

    if not np.allclose(
        cartesian.poses[0],
        start_pose,
        atol=1e-8,
    ):

        raise RuntimeError(
            "轨迹起点错误"
        )

    if not np.allclose(
        cartesian.poses[-1],
        target_pose,
        atol=1e-8,
    ):

        raise RuntimeError(
            "轨迹终点错误"
        )

    # ========================================================
    # TEST 3
    # Cartesian -> Joint
    # ========================================================

    print(
        "\n[TEST 3] "
        "Cartesian -> Joint trajectory"
    )

    # start_pose 对应的已知关节姿态
    initial_q = np.array(
        [
            30.0,
            60.0,
            20.0,
            100.0,
            0.0,
        ],
        dtype=float,
    )

    joint_trajectory = (
        planner.cartesian_to_joint(
            cartesian_trajectory=cartesian,
            initial_joint_angles_deg=initial_q,
        )
    )

    print(
        "Joint trajectory shape:",
        joint_trajectory
        .joint_angles_deg
        .shape,
    )

    print(
        "Start q:",
        np.round(
            joint_trajectory
            .joint_angles_deg[0],
            4,
        ),
    )

    print(
        "End q:",
        np.round(
            joint_trajectory
            .joint_angles_deg[-1],
            4,
        ),
    )

    print(
        "Maximum IK position error:",
        f"{joint_trajectory.max_position_error_mm:.6f} mm",
    )

    print(
        "Maximum IK orientation error:",
        f"{joint_trajectory.max_orientation_error_deg:.6f} deg",
    )

    print(
        "Maximum single joint step:",
        f"{joint_trajectory.max_joint_step_deg:.6f} deg",
    )

    if (
        joint_trajectory
        .joint_angles_deg
        .shape[1]
        != LEGACY_ROBOT.dof
    ):

        raise RuntimeError(
            "关节轨迹DOF错误"
        )

    if (
        joint_trajectory
        .max_position_error_mm
        > 0.1
    ):

        raise RuntimeError(
            "轨迹位置误差过大"
        )

    if (
        joint_trajectory
        .max_orientation_error_deg
        > 0.2
    ):

        raise RuntimeError(
            "轨迹姿态误差过大"
        )

    # ========================================================
    # TEST 4
    # FK检查最终点
    # ========================================================

    print(
        "\n[TEST 4] "
        "Final FK verification"
    )

    final_q = (
        joint_trajectory
        .joint_angles_deg[-1]
    )

    (
        final_xyz,
        _,
    ) = forward_kinematics(
        final_q
    )

    target_xyz = (
        target_pose[:3]
    )

    final_error = (
        np.linalg.norm(
            final_xyz
            - target_xyz
        )
    )

    print(
        "Target XYZ:",
        np.round(
            target_xyz,
            6,
        ),
    )

    print(
        "Actual XYZ:",
        np.round(
            final_xyz,
            6,
        ),
    )

    print(
        "Final XYZ error:",
        f"{final_error * 1000:.6f} mm",
    )

    if final_error > 1e-4:

        raise RuntimeError(
            "最终FK验证失败"
        )

    print(
        "\ntrajectory.py test passed."
    )