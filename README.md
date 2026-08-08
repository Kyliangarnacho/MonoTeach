MonoTeach

MonoTeach 是一个基于单目视觉的机械臂示教项目。项目从已有的 5DOF Legacy 机械臂出发，先建立可信的机器人运动学与 MATLAB 仿真基线，再逐步接入二维手指轨迹、三维视觉示教、轨迹重定向、安全检查和真实机械臂执行。

当前已完成：

Stage 0：5DOF Legacy Python 数学基线；

Stage 1：MATLAB 5DOF 数字机械臂基线、Python ↔ MATLAB FK 交叉验证、数值 IK 与运动演示。

目录结构

MonoTeach/
├── stage0/                         # Stage 0 Python 基线
│   ├── robot_model.py              # 5DOF 静态模型、DH、限位与默认姿态
│   ├── kinematics.py               # Modified DH、FK 与 IK
│   ├── trajectory.py               # 五次插值与轨迹规划
│   └── verify_legacy.py            # Legacy 综合验证
│
├── stage1/                         # Stage 1 MATLAB 数字机械臂
│   ├── build_legacy_robot.m        # 构建 5DOF rigidBodyTree
│   ├── plot_legacy_robot.m         # 简化机械臂几何可视化
│   ├── export_python_fk_reference.py # 导出 Stage 0 FK 参考数据
│   ├── verify_fk_consistency.m     # Python ↔ MATLAB FK 一致性验证
│   ├── verify_stage1.m             # Stage 1 综合验收入口
│   ├── demo_ik_motion.m            # IK 与机械臂运动演示
│   └── data/
│       └── python_fk_reference.json # FK 交叉验证参考数据
│
├── tests/
│   ├── test_stage0.py              # Stage 0 Python 自动化测试
│   └── test_stage1.m               # Stage 1 MATLAB 自动化测试
│
├── legacy_reference/               # 旧项目参考文件，仅作保留
├── CURRENT_STATE.md                # 当前真实工程状态
├── requirements.txt                # Python 依赖
└── pytest.ini                      # pytest 配置

stage0/output/、stage1/output/、Python 缓存和 MATLAB 临时文件均属于本地运行产物，不纳入版本控制。

Stage 0 已完成内容

5DOF 机械臂的 Modified DH 参数、关节范围和默认姿态模型；

正运动学（FK）、逆运动学（IK）与目标位姿构造；

五次多项式插值、笛卡尔轨迹与关节轨迹转换；

Legacy 综合验证；

pytest 自动化回归测试。

Stage 0 的职责是提供经过验证的 Python 机器人数学基线，后续阶段不应随意修改其 DH、FK、IK 与轨迹定义。

Stage 1 已完成内容

使用 MATLAB Robotics System Toolbox 建立与 Stage 0 一致的 5DOF rigidBodyTree；

保持相同的 Modified DH、关节顺序、关节限位和 Home Configuration；

使用简化连杆/关节几何显示机械臂，而不是只显示坐标系；

从 Stage 0 Python FK 自动导出多组参考数据；

对默认姿态、普通非零姿态和较大角度姿态进行 Python ↔ MATLAB FK 交叉验证；

使用 MATLAB inverseKinematics 求取已知可达目标的数值 IK；

通过 FK 回代验证 IK 结果；

完成 Home → Target 的机械臂运动演示与末端轨迹显示；

增加 MATLAB 自动化测试和 Stage 1 一键综合验收。

test 与 verify 的区别

tests/test_stage0.py、tests/test_stage1.m：自动化单元/回归测试，用于快速判断已有能力是否被后续修改破坏；

stage0/verify_legacy.py、stage1/verify_stage1.m：面向阶段验收的综合验证入口，会把多个模块串起来并输出更直观的阶段结果。

二者职责不同，因此同时保留。

运行

Python 环境

建议使用独立虚拟环境：

python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt

运行 Stage 0 自动化测试：

python -m pytest -q

运行 Legacy 综合验证：

python -m stage0.verify_legacy

生成 Stage 1 FK 参考数据

在项目根目录运行：

python -m stage1.export_python_fk_reference

生成：

stage1/data/python_fk_reference.json

MATLAB Stage 1

当前验证环境：MATLAB R2024a + Robotics System Toolbox。

将 MATLAB Current Folder 切换到：

<MonoTeach>/stage1

运行 FK 一致性验证：

verify_fk_consistency

运行 Stage 1 综合验收：

verify_stage1

运行 MATLAB 自动化测试：

results = runtests('../tests/test_stage1.m');
table(results)

运行 IK 运动演示：

demo_ik_motion

当前工程边界

当前尚未进入：

C920 实时视频输入；

MediaPipe 手部关键点；

二维手指轨迹示教；

ArUco / PnP 三维视觉示教；

Python ↔ MATLAB 实时通信；

碰撞与安全规划；

Simulink / Simscape Multibody；

ROS 2；

新 6DOF 机械臂；

真实机械臂执行。

下一阶段将从 Stage 2.1：C920 手部关键点与食指实时检测 开始。