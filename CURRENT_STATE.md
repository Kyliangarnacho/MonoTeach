# Current State

## 当前阶段

**Stage 3.3 CORE PASS — Position-Only Timed E2E Baseline**

Stage 3.1、Stage 3.2 与 Stage 3.3 CORE 已验收；Stage 0 / Stage 1 / Stage 2.1 / Stage 2.2 / Stage 2.3 基线保持不变。

## 已完成基线

- Stage 0：Python 5DOF Legacy 机械臂模型、Modified DH、FK、IK、轨迹规划与自动化回归。
- Stage 1：MATLAB 5DOF 数字机械臂、Python → MATLAB FK 交叉验证、数值 IK 与演示。
- Stage 2.1：C920/OpenCV、MediaPipe HandLandmarker、单手 `HandObservation` 与食指像素坐标。
- Stage 2.2：canonical immutable `raw_samples`、录制状态机、质量门控、EMA 派生轨迹、JSON 与按 `t_ms` 回放。
- Stage 2 pen-state：thumb-tip / middle-fingertip 尺度归一化 pinch、hysteresis 与防抖切换 UP / DOWN；DOWN 样本携带 semantic stroke_id。旧 trajectory 明确使用 legacy_single_stroke_default，而不是视觉自动推断。

## Stage 3.1A PASS

- Python → MATLAB WorkspaceTrajectory bridge：MATLAB 严格读取并验证 Stage 2.3 `WorkspaceTrajectory` JSON schema 1.0；fixture 覆盖 valid、invalid gap 与 valid-but-outside 语义。
- Workspace → Task Plane → Robot Base retargeting：以 190 × 290 mm workspace、默认 Task Plane 配置和 `T_base_taskplane` 将有效 workspace 点派生为 robot-base metre `TaskTrajectory`；保留 `t_ms`、invalid gap、inside/outside 证据与 source identity。
- 已完成人工 MATLAB 验收；Python 自动化回归 141 passed；`tests/test_stage3_1.m` 17 项通过；`verify_stage3_1a` PASS。

## Stage 3.1B PASS

- pre-IK eligibility：仅 `valid && inside_workspace` 样本可进入候选段；invalid gap 与 valid-but-outside 都是 barrier，保留原始 evidence 而不自动连接。
- 连续 segment 提取与可视化：输出 immutable-by-convention `segmentSet`，保留 source indices、时间范围、robot-base coordinate frame 与 metre units；各候选段独立显示，不跨 barrier 连线。
- 已完成人工 MATLAB 验收；Python 自动化回归 141 passed；`tests/test_stage3_1.m` 17 项通过；`verify_stage3_1b` PASS。

## Stage 3.2 PASS — Continuous Position-Only IK & Joint Waypoint Generation

- 建立可重复的 Position-Only IK baseline：`body5`、weights `[0 0 0 1 1 1]`、关闭 random restart；anchor IK 从 home seed 求解，并以独立 FK 回查 target XYZ。
- 定义 canonical per-point IK result contract，记录 seed/q、solver/FK 误差、显式五关节限位与 joint-limit margin；continuous IK 保留 `source_index`、`t_ms` 和 `delta_q` provenance。
- 每个 Stage 3.1 pre-IK segment 独立求解；同一 IK-success subsegment 使用 previous-success-q seed。IK failure 是保留 evidence 的 barrier，绝不伪造 q 或跨点连线；后续成功点从 `q_anchor` 重启。
- 派生 joint waypoint、FK error 与 continuity diagnostics，并提供 anchor-seed / previous-success-q seed 的诊断对照；`delta_q` 仅表示关节步长，不是速度。
- 真实三角形静态验收：91 / 91 IK success、0 failure；mean FK position error ≈ `6.689e-9 m`，max ≈ `1.642e-7 m`，overall max joint step ≈ `2.968e-2 rad`，minimum joint-limit margin ≈ `1.023 rad`。
- 验证通过：MATLAB `tests/test_stage3_2.m` 7 / 7 PASS、`verify_stage3_2` FINAL RESULT PASS、`verify_stage3_1a` / `verify_stage3_1b` PASS、完整 Python regression 141 passed。
- Stage 3.2 Position-Only solver 保持冻结；时间参数化、连续 q/qd/qdd、FK validation 与 timed animation 已由 Stage 3.3 CORE 在其上游实现。


## Stage 3.3 CORE PASS — Position-Only Timed E2E

- Minimal robot-independent boundary：CanonicalTaskTrajectory 与 RobotContext(legacy5) 已接入；RobotContext 的 velocity/acceleration 是 planning defaults，不是实测 actuator ratings。
- 真实 C920 triangle 使用冻结 candidate、2 mm task-space arc-length resampling、fresh Position-Only IK：29 / 29 success、0 failure、1 execution segment。
- TimedJointTrajectory 逐 success segment 独立生成；quintic_hermite_v1 使用确定性内部 finite-difference velocity/acceleration，并在必要时 uniform time stretch 满足 planning velocity/acceleration limits。
- 本地重新运行 E2E：duration = 4.3014898123 s；velocity / acceleration / joint limits 均 PASS；FK temporal-reference mean / max deviation = 2.9786966e-5 / 3.4453247e-4 m；minimum joint-limit margin = 0.9834568 rad。
- timed MATLAB animation 显示 Legacy5、desired Position-Only path、executed FK trail、current t_s 与 playback rate；默认标准 3D oblique view 已程序化验证，不会以 top-view 将竖直 Task Plane 压成线。
- semantic stroke foundation：当前真实 triangle 为 1 semantic stroke；semantic stroke 与 IK execution segment 分离。未来多 stroke free-space transition 尚未实现。

## Stage 3.3 DIAGNOSTIC / NEGATIVE RESULTS

- strict Writing（XYZ + body5 local-Z aiming）在真实 2 mm triangle 为 25 / 29；不进入正式 Position-Only E2E execution baseline。
- Writing generalization、Task Plane height/yaw/XY/scale sweeps、multi-start/backward rescue、orientation relaxation、Constrained Path Gap Recovery，以及 minimum-jerk / cubic diagnostics 均保留为诊断或负结果，不是正式 execution policy。

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

Stage 3.3 CORE 已冻结。下一阶段只应在明确授权后处理：multi-stroke free-space transition、collision avoidance、physical robot execution、6DOF / second robot backend、depth-camera pen-state，以及 strict Writing 29 / 29 optimization；这些均未包含在当前 PASS 中。
