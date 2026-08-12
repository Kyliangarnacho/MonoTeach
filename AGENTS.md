# MonoTeach Agent Rules

- 每次先阅读 `CURRENT_STATE.md` 和当前阶段相关文件，以真实代码与测试结果为准。
- Stage 0 / Stage 1 已冻结；无明确 bug 不修改，也不做无理由的大范围重构。
- 一个 blocker 只做少量有信息增益的验证，然后继续不依赖它的工作。
- Codex runner 当前无法访问 C920；真实摄像头由用户在普通 PowerShell 人工验收。
- `test`、`verify`、manual hardware demo 职责分开。
- 原始 trajectory / `raw_samples` 是 canonical source of truth；处理结果必须是新的派生数据，不原地修改。
- 不要误碰 `legacy_reference/final_original.py` 的既有本地修改。
- 当前阶段未结束前不擅自推进下一阶段；代码完成后运行对应 pytest 和已有 verify。
- 优先保持模块职责单一；最终汇报简述重要类/函数的输入、处理和输出。
- 除非阶段收口明确要求，否则不要自行 commit / push。
