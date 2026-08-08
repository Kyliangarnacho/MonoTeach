# Current State

**项目阶段：Stage 0 已完成**

当前仓库保留并验证了 5 自由度机械臂的 Legacy 基线实现，尚未引入 GUI、串口/PWM 控制、摄像头、ROS 或后续教学交互功能。

## 已有模块

- `stage0.robot_model`：机械臂静态参数、关节限位与默认关节姿态。
- `stage0.kinematics`：Modified DH 变换、FK、IK 与位姿误差工具。
- `stage0.trajectory`：五次插值、笛卡尔轨迹和关节轨迹规划。
- `stage0.verify_legacy`：模型、运动学和轨迹的 Legacy 基线验证；运行时生成结果文件。
- `tests.test_stage0`：Stage 0 自动化测试。
- `legacy_reference`：历史参考代码，仅作保留，不作为当前实现入口。

## 约定

- Python 依赖由 `requirements.txt` 管理，测试配置位于 `pytest.ini`。
- `.venv/`、Python 缓存、pytest 缓存及 `stage0/output/` 均为本地运行产物，不提交到 Git。
