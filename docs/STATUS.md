# 当前状态边界

以下是2026-10-19的证据快照。实验状态依据原项目持久记录，不能从仓库页面判断远程进程是否仍在运行。

| 项目 | 已知状态 | 限制 |
|---|---|---|
| ten-r21 | 历史assisted单向取出10/10 | 不是完整roundtrip |
| original020 | 独立审计OUT0/RETURN0 | raw事件不等于验收计数 |
| original019 | 已退出，exit2；原有审计正在执行 | OUT/RETURN计数尚待该审计，不另开重复审计 |
| 15物体019/021 | 两条均exit2；小文件终态记录均为capture_feedback_budget、formal step4166，raw completed_ids为空 | 原有独立审计正在处理；raw计数尚不是accepted OUT/RETURN；该对比暂未显示抓取反馈改进收益 |
| earlyhold | 002封包551项逐项校验；父端独立重跑84组CPU检查通过，78.168069 CPU秒；实际0–220命令一致，第221步首次干预可通过原有guard | 仅停止参考推进；关节仍可闭环纠偏。正式runtime/auditor集成待审核，物理收益未验证，未授权新物理实验 |
| capture-drive-contract@4 | 选定velocity target与zero guard冲突；fixed-in-tests | 全失败归因/native修复未证实 |
| capture018 actuator | 独立审计拒绝；问题unresolved | 两个raw OUT不能计accepted |
| 精简仓库 | 主树保留维护源码、单份021控制器、测试、摘要和文档，完整历史在私有Release附件 | 源码、归档、账本及macOS回归已独立验证；见provenance/VALIDATION.json |

完整目标仍是allOUT后原ID/位置/原始四元数RETURN，稳定速度、接触、磁吸解除和双臂回撤，10/15/20物体、多布局、隔离heldout，辅助条件不变且披露，报告全尝试与权威增量成本。没有full10/15/20 acceptance声明。

本地独立验证通过23项工具回归、44项当前核心回归、220个控制器绑定文件和真实历史包的6384个源文件；131条账本链及323个内容地址对象也通过校验。16个维护模块仅统一格式，AST逐一与原件一致。工程验证不产生新的物理成功结果。GitHub Actions结果以仓库Actions页面为准。

本轮源码复核进一步确认：原始接触名称筛选遗漏影响加载阶段日志；原生contact_guard独立读取场景及真实物体名，不能据此认定原生接触保护被绕过。依据和未完成审查见 `provenance/reviews/earlyhold001-source-review.json`。初始历史附件与经验摘要仍是131条记录的快照；后续权威账本继续追加，不能把旧摘要当实时状态。

提前制动002的源码与CPU复核见 `provenance/reviews/earlyhold002-source-review.json` 和 `earlyhold002-cpu-summary.json`。父端从精确封包新建副本复测，未读取改变命令之后的基线后继状态。远程生产目录的RESOURCES在封包后发生追加，已保留失败校验及归档有效的区别，见 `earlyhold002-delivery-gap.json`；下一次集成须分离不可变内容与最终资源证明。当前控制器源码仍是021，仓库文档提交不等于部署。
