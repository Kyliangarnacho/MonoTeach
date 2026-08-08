Current State

项目阶段：Stage 1 已完成

当前仓库已经建立 5DOF Legacy 机械臂的 Python / MATLAB 双端基线。Stage 0 提供经过验证的机器人数学实现；Stage 1 在 MATLAB Robotics System Toolbox 中建立同一台数字机械臂，并完成 FK 交叉验证、数值 IK、自动化测试、简化机械臂可视化和运动演示。

Stage 0 已有模块

stage0.robot_model：机械臂静态参数、Modified DH、关节限位与默认姿态；

stage0.kinematics：Modified DH 变换、FK、IK 与位姿工具；

stage0.trajectory：五次插值、笛卡尔轨迹和关节轨迹规划；

stage0.verify_legacy：Stage 0 综合验证；

tests.test_stage0：Stage 0 Python 自动化回归测试；

legacy_reference：历史参考代码，仅作保留，不作为当前实现入口。

当前 Stage 0 回归基线：

11 passed

Stage 1 已有模块

stage1/build_legacy_robot.m：根据 Stage 0 Modified DH 创建 5DOF MATLAB rigidBodyTree；

stage1/plot_legacy_robot.m：根据各关节实际坐标绘制简化连杆和关节点，使运动演示能够直观看到机械臂；

stage1/export_python_fk_reference.py：直接调用 Stage 0 FK 并导出参考数据，不重新实现第二套 Python FK；

stage1/data/python_fk_reference.json：Python ↔ MATLAB FK 交叉验证参考数据；

stage1/verify_fk_consistency.m：比较 MATLAB FK 与 Stage 0 Python FK；

stage1/verify_stage1.m：Stage 1 综合验收入口；

stage1/demo_ik_motion.m：MATLAB 数值 IK、FK 回代、机械臂动画和末端轨迹演示；

tests/test_stage1.m：Stage 1 MATLAB 自动化单元/回归测试。

Stage 1 已验证内容

MATLAB 机器人包含 5 个 revolute joints；

关节顺序、Modified DH、关节限位与 Home Configuration 与 Stage 0 一致；

Home Configuration 为 [0°, 90°, 0°, 90°, 0°]；

Python ↔ MATLAB 多组 FK 的完整 4×4 齐次变换矩阵在浮点误差范围内一致；

MATLAB inverseKinematics 可以对由合法关节角生成的可达目标求解；

IK 结果通过 FK 回代验证；

已观察到 IK 多解：求得关节角可以与构造目标时的关节角不同，但末端 Pose 一致；

MATLAB 中可以看到简化机械臂本体、关节运动和末端运动轨迹；

Stage 0 Python 测试与 Stage 1 MATLAB 测试均作为后续回归基线保留。

最近一次 IK 手工验证量级：

IK status       : success
Position error  : ~1.49e-9 m
Transform error : ~6.69e-9

Test 与 Verify 约定

test_*：自动化单元/回归测试，适合后续每次修改后快速运行；

verify_*：阶段级综合验证，将多个模块串联并输出人类可读的验收结果。

verify 不能替代自动化 test，二者都保留。

当前项目入口

Python 回归测试：

python -m pytest -q

Python Legacy 综合验证：

python -m stage0.verify_legacy

生成 Stage 1 FK Reference：

python -m stage1.export_python_fk_reference

MATLAB FK 交叉验证：

verify_fk_consistency

MATLAB Stage 1 综合验收：

verify_stage1

MATLAB Stage 1 自动化测试：

results = runtests('../tests/test_stage1.m');
table(results)

MATLAB IK Motion Demo：

demo_ik_motion

当前工程边界

目前尚未引入 GUI、串口/PWM 主控制链、C920 实时视频、MediaPipe、ArUco / PnP、Python ↔ MATLAB 实时通信、碰撞规划、Simulink、Simscape Multibody、ROS 2、新 6DOF 机械臂或真实机械臂闭环。

Stage 0 / Stage 1 现有数学基线默认冻结。后续视觉模块作为新的上游输入层接入，不应无理由修改已有 DH / FK / IK 基线。

下一阶段

下一执行阶段：

Stage 2.1 — C920 手部关键点与食指实时检测