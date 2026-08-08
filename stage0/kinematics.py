"""
kinematics.py

MonoTeach Stage 0
5DOF 机械臂运动学核心。

职责：
1. Modified DH 单节变换
2. 正运动学 FK
3. 旧项目解析 IK 参考实现
4. 带关节限位和初值的稳健数值 IK
5. (x, y, z, pitch) 目标位姿构造

不包含 GUI、串口、PWM、轨迹插值。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

from .robot_model import LEGACY_ROBOT, RobotModel


# ============================================================
# 1. IK结果数据结构
# ============================================================

@dataclass
class IKResult:
    """逆运动学求解结果。"""

    success: bool
    joint_angles_deg: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    cost: float
    message: str


# ============================================================
# 2. Modified DH
# ============================================================

def modified_dh_matrix(
    a: float,
    alpha_deg: float,
    d: float,
    theta_deg: float,
) -> np.ndarray:
    """
    生成旧项目采用的 Modified DH 齐次变换矩阵。

    参数
    ----
    a, d:
        单位 m

    alpha_deg, theta_deg:
        单位 deg
    """

    alpha = np.deg2rad(alpha_deg)
    theta = np.deg2rad(theta_deg)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    ct = np.cos(theta)
    st = np.sin(theta)

    T = np.array(
        [
            [ct, -st, 0.0, a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa, ca, ca * d],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )

    return T


# ============================================================
# 3. 正运动学
# ============================================================

def forward_transforms(
    joint_angles_deg: Iterable[float],
    robot: RobotModel = LEGACY_ROBOT,
) -> list[np.ndarray]:
    """
    计算 Base -> 每个关节坐标系的齐次变换。

    Returns
    -------
    transforms:
        [
            T0_0,
            T0_1,
            T0_2,
            ...
            T0_5
        ]
    """

    q = np.asarray(
        joint_angles_deg,
        dtype=float,
    )

    if q.shape != (robot.dof,):
        raise ValueError(
            f"关节角数量错误："
            f"期望 {robot.dof} 个，"
            f"实际 shape={q.shape}"
        )

    T = np.eye(
        4,
        dtype=float,
    )

    transforms = [
        T.copy()
    ]

    for i in range(robot.dof):

        a, alpha_deg, d = robot.dh_params[i]

        T_i = modified_dh_matrix(
            a=a,
            alpha_deg=alpha_deg,
            d=d,
            theta_deg=q[i],
        )

        T = T @ T_i

        transforms.append(
            T.copy()
        )

    return transforms


def forward_kinematics(
    joint_angles_deg: Iterable[float],
    robot: RobotModel = LEGACY_ROBOT,
) -> tuple[np.ndarray, np.ndarray]:
    """
    正运动学 FK。

    Returns
    -------
    position_m:
        [x, y, z]
        单位 m

    T_base_tool:
        Base -> Tool 的 4x4 齐次变换矩阵
    """

    transforms = forward_transforms(
        joint_angles_deg,
        robot,
    )

    T_base_tool = transforms[-1]

    position_m = (
        T_base_tool[:3, 3]
        .copy()
    )

    return (
        position_m,
        T_base_tool,
    )


def joint_positions(
    joint_angles_deg: Iterable[float],
    robot: RobotModel = LEGACY_ROBOT,
) -> np.ndarray:
    """
    获取 Base 和所有关节坐标系原点。

    后面画机械臂骨架时会直接使用。

    Returns
    -------
    shape:
        (DOF + 1, 3)
    """

    transforms = forward_transforms(
        joint_angles_deg,
        robot,
    )

    positions = np.array(
        [
            T[:3, 3]
            for T in transforms
        ],
        dtype=float,
    )

    return positions


# ============================================================
# 4. 旧项目 XYZ + Pitch -> Target Transform
# ============================================================

def target_transform_from_xyz_pitch(
    x: float,
    y: float,
    z: float,
    pitch_deg: float,
) -> np.ndarray:
    """
    按旧项目逻辑：

        x
        y
        z
        pitch

    构造目标变换矩阵。

    因为旧机械臂只有 5DOF，
    所以这里并不允许用户任意指定完整：

        roll
        pitch
        yaw

    pitch定义沿用旧项目：

        pitch = 90°
        -> 末端朝向近似竖直向下

        pitch = 0°
        -> 末端朝向近似水平向前
    """

    pitch = np.deg2rad(
        pitch_deg
    )

    radius_sq = (
        x * x
        + y * y
    )

    if radius_sq < 1e-12:
        theta1 = 0.0

    else:
        theta1 = np.arctan2(
            y,
            x,
        )

    nz = -np.sin(
        pitch
    )

    nr = np.cos(
        pitch
    )

    z_axis = np.array(
        [
            nr * np.cos(theta1),
            nr * np.sin(theta1),
            nz,
        ],
        dtype=float,
    )

    y_axis = np.array(
        [
            -np.sin(theta1),
            np.cos(theta1),
            0.0,
        ],
        dtype=float,
    )

    x_axis = np.cross(
        y_axis,
        z_axis,
    )

    # 数值归一化
    x_axis /= np.linalg.norm(
        x_axis
    )

    y_axis /= np.linalg.norm(
        y_axis
    )

    z_axis /= np.linalg.norm(
        z_axis
    )

    T_target = np.eye(
        4,
        dtype=float,
    )

    T_target[:3, 0] = x_axis
    T_target[:3, 1] = y_axis
    T_target[:3, 2] = z_axis

    T_target[:3, 3] = [
        x,
        y,
        z,
    ]

    return T_target


# ============================================================
# 5. Pose误差
# ============================================================

def rotation_error_deg(
    R_current: np.ndarray,
    R_target: np.ndarray,
) -> float:
    """
    两个旋转矩阵之间的最小旋转角误差。

    单位：
        deg
    """

    R_error = (
        R_target.T
        @ R_current
    )

    rotation = R.from_matrix(
        R_error
    )

    return float(
        np.rad2deg(
            rotation.magnitude()
        )
    )


def pose_error(
    T_current: np.ndarray,
    T_target: np.ndarray,
) -> tuple[float, float]:
    """
    计算两个Pose误差。

    Returns
    -------
    position_error_m

    orientation_error_deg
    """

    position_error = float(
        np.linalg.norm(
            T_current[:3, 3]
            - T_target[:3, 3]
        )
    )

    orientation_error = (
        rotation_error_deg(
            T_current[:3, :3],
            T_target[:3, :3],
        )
    )

    return (
        position_error,
        orientation_error,
    )


# ============================================================
# 6. Legacy解析IK
# ============================================================

def legacy_analytic_ik(
    T_target: np.ndarray,
    robot: RobotModel = LEGACY_ROBOT,
    check_limits: bool = False,
) -> np.ndarray:
    """
    旧 final.py 解析 IK 的整理版。

    作用：
        Stage 0 Callback
        保留旧算法
        与新IK进行对比

    注意：
        这里只返回一个解析分支。

        它并没有真正解决：
            IK多解
            连续轨迹选解
            最优分支选择

        所以后续正式轨迹优先使用
        inverse_kinematics()
    """

    T_target = np.asarray(
        T_target,
        dtype=float,
    )

    if T_target.shape != (4, 4):

        raise ValueError(
            "T_target 必须为 4x4"
        )

    # --------------------------------------------------------
    # 提取位置
    # --------------------------------------------------------

    px = T_target[0, 3]
    py = T_target[1, 3]
    pz = T_target[2, 3]

    XT = T_target[:3, 0]
    YT = T_target[:3, 1]
    ZT = T_target[:3, 2]

    radius_sq = (
        px ** 2
        + py ** 2
    )

    # --------------------------------------------------------
    # theta1
    # --------------------------------------------------------

    if radius_sq < 1e-12:

        M = np.array(
            [1.0, 0.0, 0.0]
        )

        theta1 = 0.0

    else:

        M = np.array(
            [
                -py,
                px,
                0.0,
            ],
            dtype=float,
        )

        M /= np.linalg.norm(
            M
        )

        theta1 = np.arctan2(
            py,
            px,
        )

    # --------------------------------------------------------
    # 将目标姿态投影到5DOF机构可表示姿态
    # --------------------------------------------------------

    K = np.cross(
        M,
        ZT,
    )

    k_norm = np.linalg.norm(
        K
    )

    if k_norm < 1e-12:

        ZT_new = ZT.copy()
        YT_new = YT.copy()
        XT_new = XT.copy()

    else:

        K /= k_norm

        ZT_new = np.cross(
            K,
            M,
        )

        ZT_new /= np.linalg.norm(
            ZT_new
        )

        cos_theta = float(
            np.clip(
                np.dot(
                    ZT,
                    ZT_new,
                ),
                -1.0,
                1.0,
            )
        )

        sin_theta = float(
            np.dot(
                np.cross(
                    ZT,
                    ZT_new,
                ),
                K,
            )
        )

        YT_new = (
            cos_theta * YT
            + sin_theta
            * np.cross(
                K,
                YT,
            )
            + (
                1.0
                - cos_theta
            )
            * np.dot(
                K,
                YT,
            )
            * K
        )

        YT_new /= np.linalg.norm(
            YT_new
        )

        XT_new = np.cross(
            YT_new,
            ZT_new,
        )

        XT_new /= np.linalg.norm(
            XT_new
        )

    T_new = np.eye(
        4,
        dtype=float,
    )

    T_new[:3, 0] = XT_new
    T_new[:3, 1] = YT_new
    T_new[:3, 2] = ZT_new

    T_new[:3, 3] = [
        px,
        py,
        pz,
    ]

    # --------------------------------------------------------
    # 提取旋转矩阵元素
    # --------------------------------------------------------

    r13 = T_new[0, 2]
    r23 = T_new[1, 2]
    r33 = T_new[2, 2]

    a11 = T_target[0, 0]
    a12 = T_target[0, 1]

    a21 = T_target[1, 0]
    a22 = T_target[1, 1]

    c1 = np.cos(
        theta1
    )

    s1 = np.sin(
        theta1
    )

    # --------------------------------------------------------
    # theta5
    # --------------------------------------------------------

    theta5 = np.arctan2(
        a21 * c1
        - a11 * s1,

        a22 * c1
        - a12 * s1,
    )

    # --------------------------------------------------------
    # theta234
    # --------------------------------------------------------

    theta234 = np.arctan2(
        r33,
        np.sqrt(
            r13 ** 2
            + r23 ** 2
        ),
    )

    # --------------------------------------------------------
    # 手腕中心
    # --------------------------------------------------------

    r_tool = np.sqrt(
        px ** 2
        + py ** 2
    )

    z_tool = pz

    # 旧机械臂几何尺寸
    d1 = 0.068
    l1 = 0.085
    l2 = 0.080
    l3 = 0.105

    r_wrist = (
        r_tool
        - l3
        * np.cos(
            theta234
        )
    )

    z_wrist = (
        z_tool
        - d1
        - l3
        * np.sin(
            theta234
        )
    )

    # --------------------------------------------------------
    # theta3 - 余弦定理
    # --------------------------------------------------------

    cos_theta3 = (
        r_wrist ** 2
        + z_wrist ** 2
        - l1 ** 2
        - l2 ** 2
    ) / (
        2.0
        * l1
        * l2
    )

    # --------------------------------------------------------
    # 修复旧代码中的一个重要问题
    #
    # 旧代码：
    #
    # np.clip(cos_theta3, -1, 1)
    #
    # 会把明显不可达目标强行裁成边界解。
    #
    # 现在：
    # 只允许浮点误差导致的微小越界。
    # --------------------------------------------------------

    if (
        cos_theta3 < -1.0 - 1e-9
        or cos_theta3 > 1.0 + 1e-9
    ):

        raise ValueError(
            "目标点超出几何可达范围："
            f"cos(theta3)="
            f"{cos_theta3:.6f}"
        )

    cos_theta3 = float(
        np.clip(
            cos_theta3,
            -1.0,
            1.0,
        )
    )

    theta3 = np.arccos(
        cos_theta3
    )

    # --------------------------------------------------------
    # theta2
    # --------------------------------------------------------

    theta2 = (
        np.arctan2(
            z_wrist,
            r_wrist,
        )
        +
        np.arctan2(
            l2
            * np.sin(
                theta3
            ),

            l1
            + l2
            * np.cos(
                theta3
            ),
        )
    )

    # --------------------------------------------------------
    # theta4
    # --------------------------------------------------------

    theta4 = (
        theta2
        - theta3
        - theta234
    )

    q_rad = np.array(
        [
            theta1,
            theta2,
            theta3,
            theta4
            + np.pi / 2.0,
            theta5,
        ],
        dtype=float,
    )

    q_deg = np.rad2deg(
        q_rad
    )

    if check_limits:

        robot.assert_joint_angles_valid(
            q_deg
        )

    return q_deg


# ============================================================
# 7. 数值IK
# ============================================================

def _ik_residual(
    q_deg: np.ndarray,
    T_target: np.ndarray,
    robot: RobotModel,
    position_scale_m: float,
    orientation_scale_rad: float,
) -> np.ndarray:
    """
    least_squares 使用的残差函数。
    """

    _, T_current = forward_kinematics(
        q_deg,
        robot,
    )

    # 位置误差
    position_residual = (
        T_current[:3, 3]
        - T_target[:3, 3]
    ) / position_scale_m

    # 姿态误差
    R_error = (
        T_target[:3, :3].T
        @ T_current[:3, :3]
    )

    orientation_residual = (
        R.from_matrix(
            R_error
        )
        .as_rotvec()
        / orientation_scale_rad
    )

    return np.concatenate(
        [
            position_residual,
            orientation_residual,
        ]
    )


def inverse_kinematics(
    T_target: np.ndarray,
    seed_deg: Optional[
        Iterable[float]
    ] = None,
    robot: RobotModel = LEGACY_ROBOT,
    position_tolerance_m: float = 1e-4,
    orientation_tolerance_deg: float = 0.2,
    max_nfev: int = 1000,
) -> IKResult:
    """
    带关节限位的数值逆运动学。

    后续轨迹规划时最重要的用法：

        上一个路径点的IK结果
                ↓
        作为下一个点的seed

    这样可以减少关节解突然跳支。
    """

    T_target = np.asarray(
        T_target,
        dtype=float,
    )

    if T_target.shape != (4, 4):

        raise ValueError(
            "T_target 必须为 4x4"
        )

    lower = (
        robot
        .joint_limits_deg[:, 0]
        .astype(float)
    )

    upper = (
        robot
        .joint_limits_deg[:, 1]
        .astype(float)
    )

    # --------------------------------------------------------
    # 初始seed
    # --------------------------------------------------------

    if seed_deg is None:

        primary_seed = (
            robot
            .default_joint_angles_deg
            .astype(float)
            .copy()
        )

    else:

        primary_seed = np.asarray(
            seed_deg,
            dtype=float,
        )

        if primary_seed.shape != (
            robot.dof,
        ):

            raise ValueError(
                f"seed_deg 必须包含 "
                f"{robot.dof} 个关节角"
            )

    eps = 1e-9

    primary_seed = np.clip(
        primary_seed,
        lower + eps,
        upper - eps,
    )

    candidate_seeds = [
        primary_seed
    ]

    # --------------------------------------------------------
    # 如果用户没有提供seed
    # 则自动尝试几个确定性初值
    # --------------------------------------------------------

    if seed_deg is None:

        candidate_seeds.extend(
            [
                (
                    robot
                    .default_joint_angles_deg
                    .astype(float)
                    .copy()
                ),

                (
                    lower
                    + upper
                ) / 2.0,

                np.array(
                    [
                        0.0,
                        60.0,
                        30.0,
                        90.0,
                        0.0,
                    ]
                ),

                np.array(
                    [
                        0.0,
                        120.0,
                        -30.0,
                        90.0,
                        0.0,
                    ]
                ),
            ]
        )

        # 旧解析IK也作为候选初值之一
        try:

            q_legacy = (
                legacy_analytic_ik(
                    T_target,
                    robot=robot,
                    check_limits=False,
                )
            )

            candidate_seeds.append(
                np.clip(
                    q_legacy,
                    lower + eps,
                    upper - eps,
                )
            )

        except Exception:
            pass

    # --------------------------------------------------------
    # 删除重复seed
    # --------------------------------------------------------

    unique_seeds = []

    for seed in candidate_seeds:

        seed = np.asarray(
            seed,
            dtype=float,
        )

        seed = np.clip(
            seed,
            lower + eps,
            upper - eps,
        )

        duplicated = any(
            np.allclose(
                seed,
                old_seed,
                atol=1e-9,
            )
            for old_seed
            in unique_seeds
        )

        if not duplicated:

            unique_seeds.append(
                seed
            )

    # --------------------------------------------------------
    # 求解
    # --------------------------------------------------------

    best_result = None

    # 残差尺度
    #
    # 约：
    # 1 mm位置误差 = 1个残差单位
    # 0.5°姿态误差 = 1个残差单位

    position_scale_m = 1e-3

    orientation_scale_rad = np.deg2rad(
        0.5
    )

    for seed in unique_seeds:

        result = least_squares(
            _ik_residual,

            x0=seed,

            bounds=(
                lower,
                upper,
            ),

            args=(
                T_target,
                robot,
                position_scale_m,
                orientation_scale_rad,
            ),

            method="trf",

            max_nfev=max_nfev,

            xtol=1e-12,
            ftol=1e-12,
            gtol=1e-12,
        )

        q_solution = (
            result.x
        )

        _, T_solution = (
            forward_kinematics(
                q_solution,
                robot,
            )
        )

        (
            position_error,
            orientation_error,
        ) = pose_error(
            T_solution,
            T_target,
        )

        candidate = IKResult(
            success=(
                result.success
                and position_error
                <= position_tolerance_m
                and orientation_error
                <= orientation_tolerance_deg
            ),

            joint_angles_deg=(
                q_solution
            ),

            position_error_m=(
                position_error
            ),

            orientation_error_deg=(
                orientation_error
            ),

            cost=float(
                result.cost
            ),

            message=str(
                result.message
            ),
        )

        if best_result is None:

            best_result = candidate
            continue

        best_score = (
            0
            if best_result.success
            else 1,

            best_result.position_error_m,

            best_result.orientation_error_deg,

            best_result.cost,
        )

        candidate_score = (
            0
            if candidate.success
            else 1,

            candidate.position_error_m,

            candidate.orientation_error_deg,

            candidate.cost,
        )

        if candidate_score < best_score:

            best_result = candidate

    assert best_result is not None

    if not best_result.success:

        best_result.message = (
            "IK未达到设定容差；"
            f"位置误差="
            f"{best_result.position_error_m * 1000:.3f} mm，"
            f"姿态误差="
            f"{best_result.orientation_error_deg:.3f} deg。"
            f"优化器信息："
            f"{best_result.message}"
        )

    return best_result


# ============================================================
# 8. XYZ + Pitch接口
# ============================================================

def solve_xyz_pitch(
    x: float,
    y: float,
    z: float,
    pitch_deg: float,
    seed_deg: Optional[
        Iterable[float]
    ] = None,
    robot: RobotModel = LEGACY_ROBOT,
) -> IKResult:
    """
    旧项目 solve_ik(x,y,z,pitch)
    的干净版本。
    """

    T_target = (
        target_transform_from_xyz_pitch(
            x=x,
            y=y,
            z=z,
            pitch_deg=pitch_deg,
        )
    )

    return inverse_kinematics(
        T_target=T_target,
        seed_deg=seed_deg,
        robot=robot,
    )


# ============================================================
# 9. Matrix打印
# ============================================================

def format_matrix(
    matrix: np.ndarray,
    decimals: int = 4,
) -> str:
    """
    将4x4矩阵格式化。
    """

    matrix = np.asarray(
        matrix,
        dtype=float,
    )

    if matrix.shape != (
        4,
        4,
    ):

        raise ValueError(
            "matrix 必须是 4x4"
        )

    cleaned = (
        matrix.copy()
    )

    cleaned[
        np.abs(cleaned)
        < 1e-12
    ] = 0.0

    return np.array2string(
        cleaned,
        precision=decimals,
        suppress_small=True,
    )


# ============================================================
# 10. 单文件测试
# ============================================================

if __name__ == "__main__":

    print(
        "=" * 64
    )

    print(
        "MonoTeach Stage 0 "
        "- kinematics.py self test"
    )

    print(
        "=" * 64
    )

    # --------------------------------------------------------
    # TEST 1：FK
    # --------------------------------------------------------

    q_default = (
        LEGACY_ROBOT
        .default_joint_angles_deg
        .copy()
    )

    (
        position,
        T_default,
    ) = forward_kinematics(
        q_default
    )

    print(
        "\n[TEST 1] "
        "Forward Kinematics"
    )

    print(
        "q =",
        q_default,
    )

    print(
        "position [m] =",
        np.round(
            position,
            6,
        ),
    )

    print(
        "T_base_tool ="
    )

    print(
        format_matrix(
            T_default
        )
    )

    # --------------------------------------------------------
    # TEST 2：FK -> IK -> FK
    # --------------------------------------------------------

    print(
        "\n[TEST 2] "
        "FK -> IK -> FK consistency"
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
                30.0,
                60.0,
                20.0,
                100.0,
                40.0,
            ]
        ),

        np.array(
            [
                -40.0,
                120.0,
                -30.0,
                80.0,
                -50.0,
            ]
        ),
    ]

    for (
        index,
        q_original,
    ) in enumerate(
        test_joint_sets,
        start=1,
    ):

        _, T_target = (
            forward_kinematics(
                q_original
            )
        )

        # 模拟真实连续轨迹：
        # seed接近上一时刻姿态
        seed = (
            q_original
            + np.array(
                [
                    5.0,
                    -3.0,
                    4.0,
                    2.0,
                    -2.0,
                ]
            )
        )

        lower = (
            LEGACY_ROBOT
            .joint_limits_deg[:, 0]
        )

        upper = (
            LEGACY_ROBOT
            .joint_limits_deg[:, 1]
        )

        seed = np.clip(
            seed,
            lower,
            upper,
        )

        result = (
            inverse_kinematics(
                T_target,
                seed_deg=seed,
            )
        )

        print(
            f"\nCase {index}"
        )

        print(
            "original q =",
            np.round(
                q_original,
                4,
            ),
        )

        print(
            "IK q       =",
            np.round(
                result.joint_angles_deg,
                4,
            ),
        )

        print(
            "position error =",
            f"{result.position_error_m * 1000:.6f} mm",
        )

        print(
            "orientation error =",
            f"{result.orientation_error_deg:.6f} deg",
        )

        print(
            "success =",
            result.success,
        )

        if not result.success:

            raise RuntimeError(
                "IK consistency test failed "
                f"at case {index}: "
                f"{result.message}"
            )

    # --------------------------------------------------------
    # TEST 3：旧解析IK
    # --------------------------------------------------------

    print(
        "\n[TEST 3] "
        "Legacy analytic IK reference"
    )

    q_reference = np.array(
        [
            30.0,
            60.0,
            20.0,
            100.0,
            40.0,
        ]
    )

    _, T_reference = (
        forward_kinematics(
            q_reference
        )
    )

    q_legacy = (
        legacy_analytic_ik(
            T_reference
        )
    )

    _, T_legacy = (
        forward_kinematics(
            q_legacy
        )
    )

    (
        position_error,
        orientation_error,
    ) = pose_error(
        T_legacy,
        T_reference,
    )

    print(
        "reference q =",
        q_reference,
    )

    print(
        "legacy IK q =",
        np.round(
            q_legacy,
            4,
        ),
    )

    print(
        "position error = "
        f"{position_error * 1000:.6f} mm"
    )

    print(
        "orientation error = "
        f"{orientation_error:.6f} deg"
    )

    if (
        position_error > 1e-4
        or orientation_error > 0.2
    ):

        raise RuntimeError(
            "Legacy analytic IK "
            "reference test failed"
        )

    print(
        "\nkinematics.py test passed."
    )