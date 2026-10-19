# 精简架构

主树承载当前实现与接口；Release归档承载可审计历史。两者由不同清单绑定，不能把历史代码当成当前实现。

| 层 | 输入与职责 | 明确边界 |
|---|---|---|
| `harness/` | Store、parent_adapter、usage_accounting | 当前科学核心，不改接口/物理政策 |
| `adapters/`、`transport/` | 活跃调度/采集/计费/观察器源码 | 原环境集成源，保留主机路径，CI不执行 |
| `experience/` | parent由实际131行快照派生的issues/regressions及引用 | 不代替完整registry或外部权威账本 |
| `archive/index.json` | private repo/tag/asset与精确大小/摘要/成员统计 | 元数据入Git，23 MiB包不入Git |
| `scripts/` | 显式下载；离线安全扫描、检查、选择恢复、测试和操作布局 | 只写忽略.local，不修旧tar/ledger |

`archive_utils.py`共享严格路径校验、JSON重复键拒绝、活动清单检查、流式扫描与选择恢复。独立硬上限是展开内容128 MiB、8000成员、单文件16 MiB，压缩包64 MiB；tar元数据/填充的解压流也有额外硬上限。拒绝绝对/穿越/反斜杠路径、重复成员、文件/目录冲突、symlink/hardlink/设备/FIFO/sparse。只接受普通文件/目录，且实际成员总数、内容字节和压缩摘要须匹配index；不会因JSON宣称安全就信任tar。

`check_repository.py`验证ACTIVE_SOURCE_MANIFEST完整覆盖活动目录；流式hash每个历史文件，核对原SOURCE_MANIFEST本身的pin和其中所有源文件；再依据原snapshot index选择所有ledger/HEAD/objects及原三个core模块到临时目录，调用原Store.rows。对象地址、链和HEAD须一致，缺失即失败；extra objects也验证地址，未登记快照和未清单化evidence拒绝。成功临时目录清理，失败保留。主树原测试必须与历史原测试字节一致。当前core允许在显式刷新活动清单后演进，不要求永远等于旧core。

`test_core.py`同样先验证活动清单和整个包的字节/源清单。仅恢复PINS、previous-evolution、incoming与全部五个.patched历史wrapper；把当前core复制进临时src，原测试复制进tests，设置模块级IN，运行恰好44项无skip的原unittest。为避免误用已加载模块，runner要求新Python进程。网络/外部进程audit hook用于测试阶段；原dispatch用mock验证接口，不启动实际任务。

`fetch_archive.py`要求现成、安全的.local/archive-cache父目录，用现有gh登录权限下载到新临时子目录。55秒超时、屏蔽gh输出、不查看环境值；验证字节/摘要后以no-clobber硬链接公布缓存。有缓存仍验证，坏缓存不替换。运行环境须支持POSIX目录与硬链接。Release尚未发布/权限不足时明确失败，不找别的来源。

`stage_operations.py`将当前3个core+11个adapter+2个transport精确复制到全新的.local/operations历史目录形状，写16文件映射与摘要，不复制ledger或配置、不替换源码绝对路径。其结果明确ready_for_live_operations=false。原生产协调器不改变；新环境集成需要parent审查后显式提供外部authority与兼容source/config。

这些工具不是任意恶意Python的安全沙箱。原Store与测试是经独立审查的可信代码；文件须来自静止checkout，避免同时编辑/替换输入。SHA仅证明身份，派生经验的语义和历史物理结论仍需独立审查。工具的成功输出明确physical_acceptance=not assessed。
