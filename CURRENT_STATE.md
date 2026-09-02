# Current State

## 当前阶段

**Stage 3.5 — Live Camera -> Simulated Robot Follow：软件链完成，最终 C920 人工验收待执行。**

- Stage 0 / 1 / 2.1 / 2.2 / 2.3 基线冻结。
- Stage 3.1、3.2、3.3 CORE、3.4 已验收；Stage 3.4 不再改动。
- Stage 3.5 的代码、确定性 TCP 流和 Python 回归已完成；最后一项是本地 C920 + MATLAB 的正常人工运行确认。

## 已冻结的关键成果

- **Stage 3.3**：Position-Only IK + continuous quintic `q(t)` + FK 验证与 Legacy5 动画。
- **Stage 3.4**：canonical `WorkspaceTrajectory2D` 经 `ReplaySource` 按 source timestamp 逐点到达，进入 rolling Cartesian planner、MATLAB IK、`q(t)` FIFO 与 100 Hz 虚拟控制器。真实 triangle：91 source events / 90 planned segments，最大 backlog `0.218 s`，ready-to-ready handoff `0 s`。
- **Stage 3.5**：C920/MediaPipe 实时观测经标定和相对映射，通过 localhost NDJSON TCP 交给 MATLAB；MATLAB 负责 Position-Only IK、q(t) FIFO、100 Hz 数字孪生与全关节可视化。

## Stage 3.5 活跃数据流

```text
C920 frame
  -> MediaPipe index fingertip
  -> undistort + selected H
  -> immutable LiveWorkspaceObservation (capture_t_ms)
  -> derived One Euro / motion gate / heartbeat
  -> formal target = measured workspace XY + mapping_offset_xy
  -> TCP: source events + committed Cartesian quintic segments
  -> MATLAB: IK -> joint q(t) FIFO -> 100 Hz simulated execution -> FK/renderer
```

- 原始 `raw_samples` / canonical trajectory 永不原地改写；滤波、时间、目标点均为派生数据。
- 两次 SPACE：第一次以稳定 P0 发起 HOME_TO_START；MATLAB 回复 `START_READY` 后，第二次建立 fresh relative mapping/source epoch 并开始教学。
- pen UP/DOWN 使用正常的 lift-transfer-lower q(t) chunks；短暂 no-hand/outside 使用 drain -> reacquire，不清空已安全入队的轨迹。
- `mapping_offset_xy` 是唯一相对映射状态；在教学开始、pen transition 完成、reacquire 完成时重置。长静止停顿会结束旧 planning run，下一段运动以干净导数窗口开始。

## 当前校准与验收证据

- 当前硬平面标定：`data/calibrations/workspace_2d/workspace_2d_20260825T171007_064501Z.json`；1280 x 720，工作区 `209 x 279 mm`。每次 live demo 必须显式传入。
- Stage 3.5 全量 Python 回归：**184 passed**。
- 近期修复：joint-quintic retiming 同时缩放 endpoint `qd/qdd`（分别除以 `s` / `s²`）；planned segment diagnostics 使用 `source_end_index`，不再出现无意义的 `NaN`。
- 最终修改后的 MATLAB batch 与 C920 人工 demo 尚未完成一次干净确认，因此 Stage 3.5 不标记为最终 PASS。

## 下一步（仅此一项）

在普通 PowerShell + MATLAB 中完成一次 C920 验收：确认两次 SPACE、平滑 HOME_TO_START、正常跟随、一次 pen/reacquire 无关节阶跃，最后收到 `FINISHED` 且 admission 为 `OPEN`，而不是 `DRAIN_TO_HOLD`。

实体机械臂、碰撞、6DOF backend、严格 Writing 姿态和绝对计量精度仍不在当前阶段范围。

## Banter Task 7 — 软件闭环完成，C920 人工验收待执行

- Task 7 增加独立的 `DISARMED/ARMED` interaction gate：双手稳定 FIVE
  仅用于控制，不进入 Grammar、Memory、Persona 或行为链；启动默认
  `DISARMED`。
- Task 4 在 Task 7 runtime 中由 Python 本地原子 execution lock 提前进入
  `DISPATCHED`；只有 MATLAB `MOTION_COMPLETED` 的完整 provenance 回执才能
  解锁。busy 时不排队、不补发。
- MATLAB 在启动时预编译十二条 Task 6 Legacy5 轨迹，并按其既有 `q(t)`
  sample clock 回放；Task 7 不改变 Task 5/6 姿态、TOPP-RA、安全限制或
  Stage 3 正式 TCP executor。
- 已通过：Banter Python 全套 **74 passed**、Task 1–7 verify、Task 5/6
  MATLAB 回归、Task 7 MATLAB 轨迹库/启动 smoke，以及 Python↔MATLAB 的
  无摄像头 `THUMBS_UP_ACK` TCP completion rehearsal。
- 仍待普通 PowerShell 人工验收：确认当前 C920 index 后启动 Task 7 demo，
  验证双手 FIVE ARM/DISARM、busy 时输入不派发、DISARM 不抢占，以及 Q
  释放相机后动作回 neutral。
