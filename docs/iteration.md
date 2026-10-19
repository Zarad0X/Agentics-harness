# 迭代、代码约定与经验

先遵守[基线优先顺序](BASELINE_FIRST.md)：从已验证ten-r21逐项扩展，以同条件端到端回归证明候选没有丢失父版本能力；发生退化时回到已验证父版本，保留失败和差异。历史单向任务与新严格往返分别验收。

每次变更使用conventional commit，如`fix: reject duplicate archive members`、`test: bind frozen fixtures to current core`、`docs: clarify external ledger setup`。一个修复对应明确问题和可复核证据，避免混入旧科学代码的外观重排。主树按用途分目录，没有通用work垃圾场；临时产物和失败恢复只进.local。

新scripts/tests/scaffold使用标准Python、显式函数、清晰命名、四空格缩进、UTF-8、末尾换行、无tab/trailing whitespace。`python3 -B scripts/check_code.py`只编译这些新源码并查空白，不安装包、不执行编译内容。pyproject提供可选Ruff lint/format范围；若开发环境已有Ruff可按显式路径运行，CI不要求它。controller/tests/core和归档保持原始字节；harness/adapters/transport已统一格式；必要科学修改以小差异、回归和真实证据审查，不以格式化遮蔽行为变化。

建议每个变更建立`docs/experiments/<distinct-id>.md`，或先用issue模板，完成后把持久记录同步到版本库：

```text
症状：实际观察和全部尝试分母。
假设：待证伪的机制，不与确认原因混写。
已确认原因/适用scope：原始证据引用与SHA-256。
基线/候选：exact source、config、runtime binding、assistance、环境。
布局：物体数/密度/seed/原目标/阈值，development或heldout。
回归：原反例、不同负例/控制例、corrective或preservation、实际/期望。
独立复核：registry revision、receipt、review、执行者与摘要。
全尝试：成功/失败/取消/预演拒绝/未收集，分别列出。
成本：wall/CPU、权威增量研发token、在线token、cached/uncached与unknown。
状态：代码 / 物理 / 多布局 / heldout / 采纳资格分别填写。
采纳或回滚：明确门槛、尚缺证据、回滚版本与触发条件。
持久经验：适用范围、不可泛化之处、外部authority追加记录ID。
```

实现者和独立审查者职责分开。完整registry义务不能因brief截断而省略；负例条件修改需显式registry revision和审查。preservation的reject/reject不等于修正。代码回归通过最多支持fixed-in-tests，不自动授权native或adoption。保持SAPIEN/PD/有限磁吸等辅助与验收标准不变；真实实验由parent另行分配资源。

原位任务须先allOUT再原始ID/位置/原四元数RETURN，稳定速度、接触、磁吸解除和双臂回撤均验收；覆盖10/15/20和多布局、隔离heldout。heldout不能回流到该次经验训练。报告全部尝试而非只选demo。计费沿用外部authority增量，resumed cumulative不是本轮成本，已观察完成但后续计费失败记accounting-unknown，未知不记0。

新经验先追加当前外部权威Store，生成不同身份的reviewed快照/归档后更新简短experience摘要与来源映射。初始`history-20261019`包不可变；新归档使用新release tag/asset身份，保留旧包、旧index和原证据。当前源的ACTIVE_SOURCE_MANIFEST是可审查更新的导入基线，不是永久代码冻结；提交旧/新hash、来源与测试。Git历史从导出开始，旧chronology在原ledger/runtime里，不伪造checkout时间或历史提交。

继承的具体经验：选定finger velocity target非零与zero guard冲突，gains/4N cap相符；生命周期残留是待证实假设，不能推导全部失败因果或native修复。capture018的两个raw OUT被独立actuator审计拒绝，不能变为accepted OUT。earlyhold缺首次触发prefix时不能只凭后续片段推定完整行为。

脚手架经验：001第一次44项测试缺collector/launcher两个.patched文件，补齐全部五个wrapper后通过，原失败已保留在001。002沿用五项冻结映射；新布局将当前core复制到临时src，而fixtures来自hash绑定的历史包。合成current-source sentinel测试用于证明没有误加载历史core，仅属工具边界测试，不是新机器人源码或native修复。归档安全测试使用小型合成证据和真实Store；具体本次结果见交付报告。
