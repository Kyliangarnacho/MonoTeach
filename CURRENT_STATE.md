# Current State

## 当前阶段

**Stage 3.1B PASS**

Stage 3.1A / Stage 3.1B 已人工验收 PASS；Stage 0 / Stage 1 / Stage 2.1 / Stage 2.2 / Stage 2.3 基线保持不变。

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

Stage 3.2：Pre-IK Segment → Continuous IK → Legacy 5DOF trajectory execution。当前尚未开始；将单独处理 IK、关节限位、连续性、速度规划与执行安全。
