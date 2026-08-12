# Current State

## 当前阶段

**Stage 2.2 软件基线已完成**

当前未推进后续阶段；Stage 2.1 已 PASS，Stage 2.2 自动化回归通过。

## 已完成基线

- Stage 0：Python 5DOF Legacy 机械臂模型、Modified DH、FK、IK、轨迹规划与自动化回归。
- Stage 1：MATLAB 5DOF 数字机械臂、Python → MATLAB FK 交叉验证、数值 IK 与演示。

## Stage 2.1 基线

- `CameraStream`：C920/OpenCV 打开、实际 profile、帧读取与单调 `timestamp_ms`。
- `HandTracker`：MediaPipe HandLandmarker `VIDEO` 模式下的单手检测。
- `HandObservation`：单帧手部观测数据契约。
- `fingertip`：normalized coordinate → pixel coordinate 的裁剪映射。
- `demo_fingertip_live`：21 点、骨架、食指 tip、坐标、FPS 与 No hand 实时显示。
- `verify_stage2_1`：不依赖摄像头的阶段软件综合验收。
- `tests/test_stage2_1.py`：无硬件的 Stage 2.1 回归测试。
- `models/README.md`：`hand_landmarker.task` 本地模型资产恢复说明；二进制文件被 Git 忽略。

## Stage 2.2 当前模块

- `trajectory` / `TrajectoryRecorder`：定义 canonical immutable `raw_samples` 及 IDLE / RECORDING / READY 录制状态机。
- `trajectory_quality`：以 normalized speed 进行 V0 质量门控，保留 invalid gap 和原始坐标。
- `trajectory_filter`：对质量门控结果生成 EMA 派生轨迹，不覆盖 raw samples。
- `trajectory_io`：保存/加载 metadata、raw samples 与 Quality Gate / EMA 配置。
- `trajectory_playback`：根据真实 `t_ms` 构造支持速度缩放的回放时间轴。
- `demo_trajectory_record`：实时比较 Raw / Filtered，支持镜像预览、alpha 切换与 JSON 保存。
- `demo_trajectory_playback`：无摄像头的 JSON 轨迹回放，支持暂停、重播和 Raw / Filtered 显示。
- `tests/test_stage2_2.py`：覆盖数据合同、Recorder、Quality Gate、EMA、JSON round-trip 与 Playback。

## 当前验证结果

- `python -m pytest -q`：70 passed。
- `python -m stage2.verify_stage2_1`：PASS。
- C920 manual live demo：PASS；左右手、21 landmarks/骨架、landmark 8、normalized/pixel 坐标、No hand 和 `q` 退出均已人工确认。
- 实时处理约 9–10 FPS。

## 已知非阻塞问题

- 长时间遮挡时 index tip 可能漂移或错误估计。
- 当前使用同步 `VIDEO` 模式，实时性能约 9–10 FPS。
- Codex runner 无法访问 C920；真实硬件验收统一由用户在普通 PowerShell 等本地环境中执行。
- `opencv-python` 与 MediaPipe 所需的 `opencv-contrib-python` 共存并共享 `cv2` 命名空间；记录为环境技术债，本阶段不清理。

## 当前工程边界

尚未实现：真实二维工作平面映射、Python ↔ MATLAB 轨迹接口、ArUco / PnP 三维示教、安全重定向、6DOF 对照、Simulink / Simscape 与实物机械臂闭环。

## 下一执行阶段

后续阶段需单独确认；当前不自动推进。
