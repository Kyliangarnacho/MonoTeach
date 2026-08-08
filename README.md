# MonoTeach

MonoTeach 是一个面向 5 自由度机械臂教学与验证的 Python 项目。当前项目完成了 Stage 0：以旧项目行为为基线，提供机械臂静态模型、运动学、轨迹规划及验证工具。

## 目录结构

```text
MonoTeach/
├── stage0/                 # Stage 0 核心模块
│   ├── robot_model.py       # 5DOF 机械臂静态模型与参数
│   ├── kinematics.py        # Modified DH、FK 与 IK
│   ├── trajectory.py        # 五次插值与轨迹规划
│   └── verify_legacy.py     # Legacy 基线验证与结果输出
├── tests/                  # Stage 0 自动化测试
├── legacy_reference/       # 保留的旧项目参考文件
├── requirements.txt        # Python 依赖
└── pytest.ini              # pytest 配置
```

`stage0/output/` 为运行 `verify_legacy` 时生成的验证产物，已被 Git 忽略。

## Stage 0 已完成内容

- 5DOF 机械臂的 Modified DH 参数、关节范围和默认姿态模型。
- 正运动学（FK）、逆运动学（IK）与目标位姿构造。
- 五次多项式插值、笛卡尔轨迹与关节轨迹转换。
- Legacy 基线验证脚本与 Stage 0 自动化测试。

## 运行

建议使用独立虚拟环境：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
pytest -q
python -m stage0.verify_legacy
```

最后一条命令会在 `stage0/output/` 中生成验证结果；该目录不纳入版本控制。
