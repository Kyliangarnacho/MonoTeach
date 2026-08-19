# MonoTeach

## 项目简介

MonoTeach 是一个单目视觉机械臂示教项目。Stage 0 和 Stage 1 建立并验证 5DOF Legacy 机械臂的 Python/MATLAB 基线；Stage 2 开始接入视觉输入。最终目标是二维/三维视觉示教、轨迹处理、机械臂仿真与实物执行。

## 当前进度

- Stage 0：完成 — Python 5DOF 机械臂数学基线。
- Stage 1：完成 — MATLAB 数字机械臂、FK 交叉验证与数值 IK。
- Stage 2.1：完成 — C920 手部关键点与食指实时检测。
- Stage 2.2：软件基线完成 — 二维轨迹录制、质量门控、EMA、JSON 持久化与按时间回放。
- Stage 2.3：完成 — C920 标定资产加载、二维工作区标定/验证，以及毫米轨迹持久化与回放。
- Stage 3.1A / 3.1B：完成 — MATLAB WorkspaceTrajectory bridge、Task Plane → Legacy robot base 几何重定向，以及 pre-IK eligibility / 连续候选段提取与可视化。
- Stage 3.2：完成 — Continuous Position-Only IK & Joint Waypoint Generation；包含 anchor/FK 回查、canonical IK result、previous-success-q 连续求解、failure barrier / q_anchor restart、关节限位与 continuity diagnostics，以及真实三角形静态验证。

## 当前目录结构

```text
MonoTeach/
├── stage0/                         # Python 5DOF 数学基线
│   ├── robot_model.py
│   ├── kinematics.py
│   ├── trajectory.py
│   └── verify_legacy.py
├── stage1/                         # MATLAB 数字机械臂基线
│   ├── build_legacy_robot.m
│   ├── plot_legacy_robot.m
│   ├── export_python_fk_reference.py
│   ├── verify_fk_consistency.m
│   ├── verify_stage1.m
│   ├── demo_ik_motion.m
│   └── data/python_fk_reference.json
├── stage2/                         # 视觉输入层
│   ├── camera_stream.py
│   ├── hand_tracker.py
│   ├── hand_observation.py
│   ├── fingertip.py
│   ├── demo_camera.py
│   ├── demo_fingertip_live.py
│   ├── trajectory.py
│   ├── trajectory_recorder.py
│   ├── trajectory_quality.py
│   ├── trajectory_filter.py
│   ├── trajectory_io.py
│   ├── trajectory_playback.py
│   ├── demo_trajectory_record.py
│   ├── demo_trajectory_playback.py
│   ├── camera_calibration.py
│   ├── workspace_geometry.py
│   ├── workspace_calibration_io.py
│   ├── calibrate_workspace_2d.py
│   ├── workspace_validation.py
│   ├── validate_workspace_2d.py
│   ├── workspace_trajectory.py
│   ├── workspace_trajectory_io.py
│   ├── convert_trajectory_to_workspace.py
│   ├── demo_workspace_trajectory_playback.py
│   ├── display_geometry.py
│   ├── verify_stage2_1.py
│   └── models/README.md
├── stage3/                         # MATLAB bridge, IK, Writing and trajectory artifacts
│   ├── config/                     # Stage 3 configuration factories
│   ├── core/                       # Workspace/task conversion and pre-IK segments
│   ├── ik/                         # Position-only and Writing IK pipelines
│   ├── writing/                    # Writing posture, placement, and feasibility tools
│   ├── resampling/                 # Arc-length trajectory artifacts
│   ├── trajectory/                 # Timing and continuous joint trajectories
│   ├── diagnostics/                # Focused analyses and visual diagnostics
│   ├── demos/                      # Manual MATLAB demos
│   ├── verification/               # Stage verification and FK validation helpers
│   └── data/workspace_trajectory_fixture.json
├── tests/
│   ├── test_stage0.py
│   ├── test_stage1.m
│   ├── test_stage2_1.py
│   ├── test_stage2_2.py
│   ├── test_stage2_3.py
│   ├── test_stage3_1.m
│   ├── test_stage3_2.m
│   └── test_stage3_3.m
├── data/                          # 本地 calibration / validation / trajectory JSON，运行时生成并被 Git 忽略
├── AGENTS.md
├── legacy_reference/               # 历史参考文件
├── CURRENT_STATE.md
├── requirements.txt
└── pytest.ini
```

`stage2/models/hand_landmarker.task` 是本地模型二进制，不提交到 Git；恢复方式见 `stage2/models/README.md`。

## Stage 0 / Stage 1 / Stage 2

- Stage 0：定义 5DOF Legacy 机械臂参数，实现 Modified DH、FK、IK、五次插值与轨迹规划。
- Stage 1：在 MATLAB Robotics System Toolbox 中建立同一机器人，完成 Python → MATLAB FK 交叉验证、数值 IK 和演示。
- Stage 2.1：接入 C920 与 MediaPipe HandLandmarker，输出单手 21 个 normalized landmarks、handedness 和食指指尖像素坐标。
- Stage 2.2：以 immutable `raw_samples` 为唯一事实源，完成录制状态机、速度质量门控、EMA 派生轨迹、JSON 保存/加载和基于 `t_ms` 的回放。
- Stage 2.3：加载 C920 K/D calibration asset，使用 `undistort_image_points()` 保持像素坐标语义；为 190 × 290 mm 工作区执行四点标定，保存 `WorkspaceCalibration` JSON（`H_image_to_workspace`）；以独立点进行毫米验证；将像素轨迹转换为 immutable `WorkspaceTrajectory2D`，并保存/加载、按毫米画布回放。
- Stage 3.1A / 3.1B：MATLAB 严格读取 Stage 2.3 WorkspaceTrajectory JSON，映射到 Task Plane / Legacy robot base metres，并仅从 `valid && inside_workspace` 样本提取连续 pre-IK candidate segments；invalid 与 outside evidence 保留且都是 segment barrier。
- Stage 3.2：以 deterministic Position-Only baseline 为每个 candidate target 求解 Legacy 5DOF IK；anchor 与每个成功点均有 FK XYZ 回查、显式 joint-limit / margin evidence 和 canonical result contract。连续成功点由 previous-success-q seed，failure 保留为 barrier 且后续从 q_anchor restart；输出带 source/time provenance 的 joint waypoints 与只读 `delta_q` continuity diagnostics。真实三角形静态验收为 91 / 91 success、0 failure，mean / max FK error 约 `6.689e-9` / `1.642e-7 m`。

当前 independent validation baseline：mean ≈ 1.534 mm、RMS ≈ 1.696 mm、max ≈ 2.502 mm。验证真值为人工粗略量取，仅作为当前 baseline，不作为高精度计量结论。

Stage 2.1 数据流：

```text
C920
  → CameraStream
  → BGR frame + timestamp_ms
  → HandTracker / MediaPipe HandLandmarker
  → HandObservation
  → index fingertip normalized / pixel coordinate
  → live demo
```

## 运行方式

在项目根目录创建并启用虚拟环境后安装依赖：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Python 自动化回归：

```powershell
python -m pytest -q
```

Stage 0 综合验证：

```powershell
python -m stage0.verify_legacy
```

Stage 1 Python FK 参考数据导出：

```powershell
python -m stage1.export_python_fk_reference
```

Stage 1 MATLAB 入口（将 Current Folder 切换到 `stage1/`）：

```matlab
verify_fk_consistency
verify_stage1
demo_ik_motion
```

Stage 2.1 软件验收和实时 Demo：

```powershell
python -m stage2.verify_stage2_1
python -m stage2.demo_camera
python -m stage2.demo_fingertip_live
```

真实摄像头 Demo 需要在普通本地 PowerShell 等具有摄像头权限的环境中运行。

Stage 2.2 轨迹录制与回放：

```powershell
python -m stage2.demo_trajectory_record
python -m stage2.demo_trajectory_playback "data\trajectories\<trajectory>.json"
```

录制 Demo 使用 `Space` 开始/停止、`S` 保存、`P` 显示回放命令；回放 Demo 使用 `Space` 暂停/继续、`R` 重播、`Q` 退出。可用 `--speed 0.5|1.0|2.0` 和 `--show-raw` 调整回放。

Stage 2.3 工作区标定、验证与毫米轨迹：

```powershell
python -m stage2.camera_calibration <camera_params.npz>
python -m stage2.calibrate_workspace_2d --camera-calibration <camera_params.npz> --width-mm 190 --height-mm 290 --camera-index <actual-index>
python -m stage2.validate_workspace_2d --camera-calibration <camera_params.npz> --workspace-calibration <workspace.json> --camera-index <actual-index>
python -m stage2.convert_trajectory_to_workspace --trajectory <trajectory.json> --camera-calibration <camera_params.npz> --workspace-calibration <workspace.json>
python -m stage2.demo_workspace_trajectory_playback <workspace_trajectory.json> --speed 1.0
```

OpenCV camera index 不是稳定硬件 ID；Windows 重新枚举后应先确认实际设备/index，再运行真实硬件入口。

Stage 3.1 MATLAB bridge / retargeting（在仓库根目录）：

```matlab
addpath(genpath(fullfile(pwd, 'stage3')))
verify_stage3_1a
verify_stage3_1b
results = runtests('tests/test_stage3_1.m'); disp(table(results))
```

Stage 3.2 Continuous Position-Only IK / joint waypoints（在仓库根目录）：

```matlab
addpath(genpath(fullfile(pwd, 'stage3')))
addpath(fullfile(pwd, 'stage1'))
verify_stage3_2
results = runtests('tests/test_stage3_2.m'); disp(table(results))
demo_continuous_ik('data/workspace_trajectories/<workspace_trajectory>.json')
demo_ik_waypoint_snapshots('data/workspace_trajectories/<workspace_trajectory>.json')
```

## Test / Verify / Manual Demo

- `test`：自动化单元与回归测试，不依赖真实摄像头。
- `verify`：阶段级软件综合验收，不打开真实摄像头。
- `manual demo`：真实硬件人工验收，包括画面、手部检测、可视化和退出行为。

## 当前工程边界 / 后续方向

C920、二维工作区、毫米轨迹和 MATLAB Position-Only continuous IK / joint waypoints 已接入。当前尚未实现：

- 最终书写姿态约束、时间参数化、qdot/qddot、timed animation、碰撞与 Legacy 5DOF 执行安全。
- ArUco / PnP 三维示教。
- 安全轨迹重定向。
- 6DOF 对照、Simulink / Simscape 与实物机械臂闭环。
