"""
robot_model.py

MonoTeach Stage 0
旧 5DOF 机械臂的“静态模型定义”。

这个文件只负责描述机械臂本身：
1. 自由度
2. Modified DH 参数
3. 关节限位
4. 默认关节角
5. 一些基础模型检查

注意：
这里不写 FK、IK、轨迹规划。
这些算法分别放到 kinematics.py 和 trajectory.py。
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np


# ============================================================
# 1. 基础信息
# ============================================================

ROBOT_NAME = "Legacy_5DOF_Arm"

DOF = 5

DH_CONVENTION = "modified_dh"


# ============================================================
# 2. Modified DH 参数
# ============================================================
#
# 参数顺序：
# (a_{i-1}, alpha_{i-1}, d_i)
#
# 长度单位：m
# 角度单位：deg
#
# 来自旧课程设计：
#
# Joint 1: a = 0      alpha = 0°      d = 0.068
# Joint 2: a = 0      alpha = 90°     d = 0
# Joint 3: a = 0.085  alpha = 180°    d = 0
# Joint 4: a = 0.080  alpha = 0°      d = 0
# Joint 5: a = 0      alpha = 90°     d = 0.105
#
# theta 是变量，因此不写死在这里。
# ============================================================

DH_PARAMS = np.array(
    [
        [0.000,   0.0,   0.068],
        [0.000,  90.0,   0.000],
        [0.085, 180.0,   0.000],
        [0.080,   0.0,   0.000],
        [0.000,  90.0,   0.105],
    ],
    dtype=float,
)


# ============================================================
# 3. 关节限位
# ============================================================
#
# 与旧 final.py 中 SERVO_CONFIG 保持一致。
#
# 每行：
# [最小角度, 最大角度]
#
# 单位：deg
# ============================================================

JOINT_LIMITS_DEG = np.array(
    [
        [-120.0, 120.0],   # J1
        [   0.0, 180.0],   # J2
        [-120.0, 120.0],   # J3
        [ -30.0, 210.0],   # J4
        [-120.0, 120.0],   # J5
    ],
    dtype=float,
)


# ============================================================
# 4. 默认关节状态
# ============================================================
#
# 旧项目中经常使用的初始姿态：
#
# J1 =   0°
# J2 =  90°
# J3 =   0°
# J4 =  90°
# J5 =   0°
#
# 这里只把它定义为软件验证使用的默认姿态，
# 暂时不要把它理解成严格的机器人“机械零位”。
# ============================================================

DEFAULT_JOINT_ANGLES_DEG = np.array(
    [0.0, 90.0, 0.0, 90.0, 0.0],
    dtype=float,
)


# ============================================================
# 5. 舵机原始配置
# ============================================================
#
# 保留旧项目数据，仅用于 Legacy Callback。
#
# 格式：
# joint_index:
# (
#     pwm_at_min_or_reference,
#     pwm_at_max_or_reference,
#     min_angle,
#     max_angle,
#     reference_angle
# )
#
# Stage 0 不会直接使用 PWM 控制。
# ============================================================

SERVO_CONFIG = {
    0: (1500, 2500, -120.0, 120.0, 0.0),
    1: (750, 1500, 0.0, 180.0, 90.0),
    2: (1500, 2250, -120.0, 120.0, 0.0),
    3: (750, 1500, -30.0, 210.0, 90.0),
    4: (1500, 2500, -120.0, 120.0, 0.0),
}


# ============================================================
# 6. RobotModel 数据类
# ============================================================

@dataclass(frozen=True)
class RobotModel:
    """
    保存机械臂的静态参数。

    这里没有任何运动学计算。
    """

    name: str
    dof: int
    dh_params: np.ndarray
    joint_limits_deg: np.ndarray
    default_joint_angles_deg: np.ndarray
    dh_convention: str = "modified_dh"

    def validate(self) -> None:
        """
        检查模型参数本身是否合法。
        """

        if self.dof <= 0:
            raise ValueError("DOF 必须大于 0")

        if self.dh_params.shape != (self.dof, 3):
            raise ValueError(
                f"DH 参数尺寸错误："
                f"期望 {(self.dof, 3)}，"
                f"实际 {self.dh_params.shape}"
            )

        if self.joint_limits_deg.shape != (self.dof, 2):
            raise ValueError(
                f"关节限位尺寸错误："
                f"期望 {(self.dof, 2)}，"
                f"实际 {self.joint_limits_deg.shape}"
            )

        if self.default_joint_angles_deg.shape != (self.dof,):
            raise ValueError(
                f"默认关节角尺寸错误："
                f"期望 {(self.dof,)}，"
                f"实际 {self.default_joint_angles_deg.shape}"
            )

        for i in range(self.dof):
            joint_min = self.joint_limits_deg[i, 0]
            joint_max = self.joint_limits_deg[i, 1]

            if joint_min >= joint_max:
                raise ValueError(
                    f"J{i + 1} 关节限位错误："
                    f"{joint_min} >= {joint_max}"
                )

        if not self.are_joint_angles_valid(
            self.default_joint_angles_deg
        ):
            raise ValueError(
                "默认关节角超出关节限位"
            )

    def are_joint_angles_valid(
        self,
        joint_angles_deg
    ) -> bool:
        """
        判断一组关节角是否全部处于允许范围内。

        Parameters
        ----------
        joint_angles_deg:
            长度为 DOF 的关节角数组，单位 deg。

        Returns
        -------
        bool
        """

        angles = np.asarray(
            joint_angles_deg,
            dtype=float
        )

        if angles.shape != (self.dof,):
            return False

        lower = self.joint_limits_deg[:, 0]
        upper = self.joint_limits_deg[:, 1]

        return bool(
            np.all(angles >= lower)
            and np.all(angles <= upper)
        )

    def assert_joint_angles_valid(
        self,
        joint_angles_deg
    ) -> np.ndarray:
        """
        检查关节角是否合法。

        合法：
            返回 numpy 数组。

        非法：
            抛出 ValueError。
        """

        angles = np.asarray(
            joint_angles_deg,
            dtype=float
        )

        if angles.shape != (self.dof,):
            raise ValueError(
                f"关节角数量错误："
                f"期望 {self.dof} 个，"
                f"实际 shape={angles.shape}"
            )

        for i, angle in enumerate(angles):
            lower, upper = self.joint_limits_deg[i]

            if not lower <= angle <= upper:
                raise ValueError(
                    f"J{i + 1} = {angle:.3f}° "
                    f"超出限位 "
                    f"[{lower:.1f}°, {upper:.1f}°]"
                )

        return angles

    def clip_joint_angles(
        self,
        joint_angles_deg
    ) -> np.ndarray:
        """
        将关节角强制裁剪到允许范围。

        注意：
        后续正式轨迹规划阶段不能靠 clip
        来掩盖不可达问题。

        这个函数主要用于调试和安全保护。
        """

        angles = np.asarray(
            joint_angles_deg,
            dtype=float
        )

        if angles.shape != (self.dof,):
            raise ValueError(
                f"关节角数量必须为 {self.dof}"
            )

        lower = self.joint_limits_deg[:, 0]
        upper = self.joint_limits_deg[:, 1]

        return np.clip(
            angles,
            lower,
            upper
        )

    def get_dh_row(
        self,
        joint_index: int
    ) -> Tuple[float, float, float]:
        """
        获取指定关节的 DH 参数。

        joint_index 使用 Python 索引：
        0 ~ 4

        Returns
        -------
        a, alpha_deg, d
        """

        if not 0 <= joint_index < self.dof:
            raise IndexError(
                f"joint_index 必须位于 "
                f"0 ~ {self.dof - 1}"
            )

        a, alpha_deg, d = self.dh_params[joint_index]

        return (
            float(a),
            float(alpha_deg),
            float(d),
        )

    def print_summary(self) -> None:
        """
        在终端打印机器人模型摘要。
        """

        print("=" * 60)
        print(f"Robot Name : {self.name}")
        print(f"DOF        : {self.dof}")
        print(f"DH Type    : {self.dh_convention}")
        print("=" * 60)

        print("\nModified DH Parameters")
        print(
            "Joint | "
            "a_{i-1} (m) | "
            "alpha_{i-1} (deg) | "
            "d_i (m)"
        )

        for i in range(self.dof):
            a, alpha, d = self.dh_params[i]

            print(
                f"J{i + 1:<4} | "
                f"{a:11.3f} | "
                f"{alpha:17.1f} | "
                f"{d:7.3f}"
            )

        print("\nJoint Limits")

        for i in range(self.dof):
            lower, upper = self.joint_limits_deg[i]

            print(
                f"J{i + 1}: "
                f"[{lower:7.1f}, "
                f"{upper:7.1f}] deg"
            )

        print(
            "\nDefault Joint Angles:"
        )

        print(
            self.default_joint_angles_deg
        )

        print("=" * 60)


# ============================================================
# 7. 创建 Stage 0 唯一正式机械臂模型实例
# ============================================================

LEGACY_ROBOT = RobotModel(
    name=ROBOT_NAME,
    dof=DOF,
    dh_params=DH_PARAMS.copy(),
    joint_limits_deg=JOINT_LIMITS_DEG.copy(),
    default_joint_angles_deg=DEFAULT_JOINT_ANGLES_DEG.copy(),
    dh_convention=DH_CONVENTION,
)


# 导入模块时立即检查模型数据。
LEGACY_ROBOT.validate()


# ============================================================
# 8. 单文件测试
# ============================================================

if __name__ == "__main__":

    LEGACY_ROBOT.print_summary()

    print("\n[TEST 1] 默认姿态是否合法？")
    print(
        LEGACY_ROBOT.are_joint_angles_valid(
            DEFAULT_JOINT_ANGLES_DEG
        )
    )

    print("\n[TEST 2] 故意输入非法姿态：")

    invalid_angles = np.array(
        [200.0, 90.0, 0.0, 90.0, 0.0]
    )

    print(
        LEGACY_ROBOT.are_joint_angles_valid(
            invalid_angles
        )
    )

    print("\nrobot_model.py test passed.")