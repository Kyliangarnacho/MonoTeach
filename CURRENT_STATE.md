# Current State

## 当前阶段

**Stage 3.2 PASS — Continuous Position-Only IK & Joint Waypoint Generation**

Stage 3.1 与 Stage 3.2 已人工验收 PASS；Stage 0 / Stage 1 / Stage 2.1 / Stage 2.2 / Stage 2.3 基线保持不变。

## 已完成基线

- Stage 0：Python 5DOF Legacy 机械臂模型、Modified DH、FK、IK、轨迹规划与自动化回归。
- Stage 1：MATLAB 5DOF 数字机械臂、Python → MATLAB FK 交叉验证、数值 IK 与演示。
- Stage 2.1：C920/OpenCV、MediaPipe HandLandmarker、单手 `HandObservation` 与食指像素坐标。
- Stage 2.2：canonical immutable `raw_samples`、录制状态机、质量门控、EMA 派生轨迹、JSON 与按 `t_ms` 回放。

## Stage 3.1A PASS

- Python → MATLAB WorkspaceTrajectory bridge：MATLAB 严格读取并验证 Stage 2.3 `WorkspaceTrajectory` JSON schema 1.0；fixture 覆盖 valid、invalid gap 与 valid-but-outside 语义。
- Workspace → Task Plane → Robot Base retargeting：以 190 × 290 mm workspace、默认 Task Plane 配置和 `T_base_taskplane` 将有效 workspace 点派生为 robot-base metre `TaskTrajectory`；保留 `t_ms`、invalid gap、inside/outside 证据与 source identity。
- 已完成人工 MATLAB 验收；Python 自动化回归 138 passed；`tests/test_stage3_1.m` 13 项通过；`verify_stage3_1a` PASS。

## Stage 3.1B PASS

- pre-IK eligibility：仅 `valid && inside_workspace` 样本可进入候选段；invalid gap 与 valid-but-outside 都是 barrier，保留原始 evidence 而不自动连接。
- 连续 segment 提取与可视化：输出 immutable-by-convention `segmentSet`，保留 source indices、时间范围、robot-base coordinate frame 与 metre units；各候选段独立显示，不跨 barrier 连线。
- 已完成人工 MATLAB 验收；Python 自动化回归 138 passed；`tests/test_stage3_1.m` 13 项通过；`verify_stage3_1b` PASS。

## Stage 3.2 PASS — Continuous Position-Only IK & Joint Waypoint Generation

- 建立可重复的 Position-Only IK baseline：`body5`、weights `[0 0 0 1 1 1]`、关闭 random restart；anchor IK 从 home seed 求解，并以独立 FK 回查 target XYZ。
- 定义 canonical per-point IK result contract，记录 seed/q、solver/FK 误差、显式五关节限位与 joint-limit margin；continuous IK 保留 `source_index`、`t_ms` 和 `delta_q` provenance。
- 每个 Stage 3.1 pre-IK segment 独立求解；同一 IK-success subsegment 使用 previous-success-q seed。IK failure 是保留 evidence 的 barrier，绝不伪造 q 或跨点连线；后续成功点从 `q_anchor` 重启。
- 派生 joint waypoint、FK error 与 continuity diagnostics，并提供 anchor-seed / previous-success-q seed 的诊断对照；`delta_q` 仅表示关节步长，不是速度。
- 真实三角形静态验收：91 / 91 IK success、0 failure；mean FK position error ≈ `6.689e-9 m`，max ≈ `1.642e-7 m`，overall max joint step ≈ `2.968e-2 rad`，minimum joint-limit margin ≈ `1.023 rad`。
- 验证通过：MATLAB `tests/test_stage3_2.m` 7 / 7 PASS、`verify_stage3_2` FINAL RESULT PASS、`verify_stage3_1a` / `verify_stage3_1b` PASS、完整 Python regression 138 passed。
- 当前仍是 Position-Only baseline：最终书写姿态约束、时间参数化、qdot/qddot、timed animation、碰撞与执行安全尚未实现。

## Stage 2.3

- `camera_calibration`：加载 C920 NPZ/OpenCV YAML K/D calibration asset，并对稀疏像素点执行 `undistort_image_points()`。
- `workspace_geometry` / `workspace_calibration_io`：定义 190 × 290 mm `WorkspaceDefinition`、`WorkspaceCalibration` 和 JSON artifact；四点标定使用 undistorted image pixels 计算 `H_image_to_workspace`。
- `workspace_validation`：独立验证链为 raw image pixel → undistorted pixel → `H_image_to_workspace` → workspace mm，不重新拟合 H。
- 当前 independent validation baseline：mean ≈ 1.534 mm、RMS ≈ 1.696 mm、max ≈ 2.502 mm。ground truth 为人工粗略量取，仅作为当前 baseline，不作为高精度计量结论。
- `workspace_trajectory` / `workspace_trajectory_io`：将 Stage 2.2 像素轨迹派生为 immutable `WorkspaceTrajectory2D`；保留 `t_ms`、invalid gap 和工作区外映射证据，并保存/加载 JSON。
- `demo_workspace_trajectory_playback`：在保持真实工作区长宽比例的毫米画布上回放 workspace trajectory。
- 真实 Stage 2.3 E2E manual validation：PASS。

## 硬件与运行注意事项

- Codex runner 无法访问 C920；真实硬件验收由用户在普通 PowerShell 等本地环境中运行。
- OpenCV camera index 不是稳定硬件 ID；Windows 重新枚举可能改变 C920 index。硬件阶段开始前确认实际设备/index，不要仅依据历史 index 误判 backend、resolution 或代码失败。
- `opencv-python` 与 MediaPipe 所需的 `opencv-contrib-python` 共存并共享 `cv2` 命名空间；记录为环境技术债，本阶段不清理。

## 下一执行阶段

Stage 3.3：在已验收的 Position-Only joint waypoints 之上，定义最终书写姿态约束与安全执行准备；时间参数化、qdot/qddot、timed animation、碰撞与执行安全仍待单独设计和验证。
