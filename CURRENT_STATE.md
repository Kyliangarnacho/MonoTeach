# Current State

## 当前阶段

**Stage 2.1 已完成 / PASS**

下一阶段：**Stage 2.2 — 二维食指轨迹记录、滤波与回放**。

## 已完成基线

- Stage 0：Python 5DOF Legacy 机械臂模型、Modified DH、FK、IK、轨迹规划与自动化回归。
- Stage 1：MATLAB 5DOF 数字机械臂、Python → MATLAB FK 交叉验证、数值 IK 与演示。

## Stage 2.1 当前模块

- `CameraStream`：C920/OpenCV 打开、实际 profile、帧读取与单调 `timestamp_ms`。
- `HandTracker`：MediaPipe HandLandmarker `VIDEO` 模式下的单手检测。
- `HandObservation`：单帧手部观测数据契约。
- `fingertip`：normalized coordinate → pixel coordinate 的裁剪映射。
- `demo_fingertip_live`：21 点、骨架、食指 tip、坐标、FPS 与 No hand 实时显示。
- `verify_stage2_1`：不依赖摄像头的阶段软件综合验收。
- `tests/test_stage2_1.py`：无硬件的 Stage 2.1 回归测试。
- `models/README.md`：`hand_landmarker.task` 本地模型资产恢复说明；二进制文件被 Git 忽略。

## 当前验证结果

- `python -m pytest -q`：21 passed。
- `python -m stage2.verify_stage2_1`：PASS。
- C920 manual live demo：PASS；左右手、21 landmarks/骨架、landmark 8、normalized/pixel 坐标、No hand 和 `q` 退出均已人工确认。
- 实时处理约 9–10 FPS。

## 已知非阻塞问题

- 长时间遮挡时 index tip 可能漂移或错误估计。
- 当前使用同步 `VIDEO` 模式，实时性能约 9–10 FPS。
- Codex runner 无法访问 C920；真实硬件验收统一由用户在普通 PowerShell 等本地环境中执行。
- `opencv-python` 与 MediaPipe 所需的 `opencv-contrib-python` 共存并共享 `cv2` 命名空间；记录为环境技术债，本阶段不清理。

## 当前工程边界

Stage 2.2 及以后尚未实现：二维轨迹记录/滤波/回放、真实工作平面映射、Python ↔ MATLAB 轨迹接口、ArUco / PnP 三维示教、安全重定向、6DOF 对照、Simulink / Simscape 与实物机械臂闭环。

## 下一执行阶段

**Stage 2.2 — 二维食指轨迹记录、滤波与回放**。
