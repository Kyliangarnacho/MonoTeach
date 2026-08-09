# MonoTeach

## 项目简介

MonoTeach 是一个单目视觉机械臂示教项目。Stage 0 和 Stage 1 建立并验证 5DOF Legacy 机械臂的 Python/MATLAB 基线；Stage 2 开始接入视觉输入。最终目标是二维/三维视觉示教、轨迹处理、机械臂仿真与实物执行。

## 当前进度

- Stage 0：完成 — Python 5DOF 机械臂数学基线。
- Stage 1：完成 — MATLAB 数字机械臂、FK 交叉验证与数值 IK。
- Stage 2.1：完成 — C920 手部关键点与食指实时检测。
- 下一阶段：Stage 2.2 — 二维食指轨迹记录、滤波与回放。

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
│   ├── verify_stage2_1.py
│   └── models/README.md
├── tests/
│   ├── test_stage0.py
│   ├── test_stage1.m
│   └── test_stage2_1.py
├── legacy_reference/               # 历史参考文件
├── CURRENT_STATE.md
├── requirements.txt
└── pytest.ini
```

`stage2/models/hand_landmarker.task` 是本地模型二进制，不提交到 Git；恢复方式见 `stage2/models/README.md`。

## Stage 0 / Stage 1 / Stage 2.1

- Stage 0：定义 5DOF Legacy 机械臂参数，实现 Modified DH、FK、IK、五次插值与轨迹规划。
- Stage 1：在 MATLAB Robotics System Toolbox 中建立同一机器人，完成 Python → MATLAB FK 交叉验证、数值 IK 和演示。
- Stage 2.1：接入 C920 与 MediaPipe HandLandmarker，输出单手 21 个 normalized landmarks、handedness 和食指指尖像素坐标。

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

## Test / Verify / Manual Demo

- `test`：自动化单元与回归测试，不依赖真实摄像头。
- `verify`：阶段级软件综合验收，不打开真实摄像头。
- `manual demo`：真实硬件人工验收，包括画面、手部检测、可视化和退出行为。

## 当前工程边界 / 后续方向

C920 与 MediaPipe 已接入。当前尚未实现：

- 二维轨迹记录、滤波与回放。
- 真实二维工作平面映射。
- Python ↔ MATLAB 轨迹接口。
- ArUco / PnP 三维示教。
- 安全轨迹重定向。
- 6DOF 对照、Simulink / Simscape 与实物机械臂闭环。
