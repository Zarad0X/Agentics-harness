# 当前状态边界

以下是2026-10-19的证据快照。实验状态依据原项目持久记录，不能从仓库页面判断远程进程是否仍在运行。

| 项目 | 已知状态 | 限制 |
|---|---|---|
| ten-r21 | 历史assisted单向取出10/10 | 不是完整roundtrip |
| original020 | 独立审计OUT0/RETURN0 | raw事件不等于验收计数 |
| original019 | 独立审计OUT1/RETURN0；终止于capture_both_contacts_lost | 诊断部分取出，完整任务失败；未完成RETURN与最终安全验收 |
| 15物体019/021 | 两条独立审计均OUT0/15、精确RETURN0/15，完整失败；均为capture_feedback_budget、正式第4166步 | 同一seed101配置与辅助条件；这组没有显示021的收益。仅一个开发布局，CPU诊断；已有流程负责后续复盘 |
| earlyhold | 002封包551项逐项校验；父端独立重跑84组CPU检查通过，78.168069 CPU秒；实际0–220命令一致，第221步首次干预可通过原有guard | 仅停止参考推进；关节仍可闭环纠偏。正式023/024集成已父端复核；48个实际registry用例与调用链检查通过，新预算共约102CPU秒。各一条原始10物体诊断已启动，物理收益未验证 |
| capture-drive-contract@4 | 选定velocity target与zero guard冲突；fixed-in-tests | 全失败归因/native修复未证实 |
| capture018 actuator | 独立审计拒绝；问题unresolved | 两个raw OUT不能计accepted |
| 精简仓库 | 主树保留维护源码、单份024控制器、测试、摘要和文档，完整历史在私有Release附件 | 源码、归档、账本及macOS回归已独立验证；见provenance/VALIDATION.json |

完整目标仍是allOUT后原ID/位置/原始四元数RETURN，稳定速度、接触、磁吸解除和双臂回撤，10/15/20物体、多布局、隔离heldout，辅助条件不变且披露，报告全尝试与权威增量成本。没有full10/15/20 acceptance声明。

初次本地独立验证通过23项工具回归、44项当前核心回归、当时220个控制器绑定文件和真实历史包的6384个源文件；131条账本链及323个内容地址对象也通过校验。16个维护模块仅统一格式，AST逐一与原件一致。工程验证不产生新的物理成功结果。GitHub Actions结果以仓库Actions页面为准。

本轮源码复核进一步确认：原始接触名称筛选遗漏影响加载阶段日志；原生contact_guard独立读取场景及真实物体名，不能据此认定原生接触保护被绕过。依据和未完成审查见 `provenance/reviews/earlyhold001-source-review.json`。初始历史附件与经验摘要仍是131条记录的快照；后续权威账本继续追加，不能把旧摘要当实时状态。

提前制动002的源码与CPU复核见 `provenance/reviews/earlyhold002-source-review.json` 和 `earlyhold002-cpu-summary.json`。父端从精确封包新建副本复测，未读取改变命令之后的基线后继状态。远程生产目录的RESOURCES在封包后发生追加，已保留失败校验及归档有效的区别，见 `earlyhold002-delivery-gap.json`；下一次集成须分离不可变内容与最终资源证明。当前控制器已更新为父端审查后的024（225个绑定文件）；023/024均已另经原受控流程启动诊断。Git提交本身不触发部署。

原始019/020复盘已接收并经父端16组CPU复测，诊断计数分别为1/0与0/0，两条均失败。023/024仅改变提前制动开关及身份，七份配置、020抓取、磁吸与安全判定不变；父端独立补齐几何证据，不改写生产任务原有超时失败。详见 `provenance/reviews/postmortem019020-review.json`、`earlyhold-native001-source-review.json`、`earlyhold-native001-cpu-summary.json` 和 `earlyhold-original10-experiment.json`。新对比预算为每版一条、无自动重试，终局必须独立审计。

019的N4失败证据缺口已补齐：正式失败链1995条事件，同一候选两条成功预演分别2594/2682条，共7271条；源事件文件哈希、六个源码哈希和每条链的序号/前驱哈希均已核验。原始166MB日志及压缩数据不进主库，引用见 `provenance/reviews/capture019-chain-validation.json`。22号机Astra完成诊断后，父端核对167份原始019运行文件并独立复测16组测试（6.938922 CPU秒）。逐条接触谓词复算确认：N4只有finger0在443–489步取得有效支撑，finger1始终未取得；490步唯一支撑消失，原恢复保护在发出恢复命令前正确拒绝。因此失败标签不证明“两侧已夹稳后同时松脱”。接触消失的物理原因仍未知，没有生产修复补丁。详见 `provenance/reviews/capture019-diagnosis-review.json`。

正式失败链能在明确条件下用原控制器与原审计器回放；两次预演先前只回放到缺少完整closure_source输入的位置。父端随后从同一封闭日志取回全部相邻控制记录：102/168条，共270条，原有capture事件逐项不变。新补齐的是历史输入；几何证书和成功guard回调仍是历史API条件，不是重新运行的原生几何或物理证明。22号机Astra正在独立目录补做完整预演回放（新180 CPU秒预算）；无额外物理尝试。数据身份与边界见 `capture019-preview-contiguous-summary.json`。

023关闭提前制动的诊断已exit2，raw失败为loaded_slip_insufficient_braking_headroom，原有观察器已完成独立审计，OUT0/10、精确RETURN0/10，凭据有效但完整失败；024开启版尚在运行。不得从运行时间更长推断成功或稳定收益。原配对仍为每版一条，无重试。15物体两份独立审计、配置/运行绑定及全部尝试的负结果已父端核对，见 `n15-feedbackpair-result-review.json`；既有观察器已启动其专属复盘，未另开重复审计。

父端已独立完成019完整历史回放：1995条正式事件与2696/2850条预演事件全部匹配；16组基础、16个辅助输入反例、11个独立凭据用例通过，共23.783090 CPU秒。167份运行文件保持原019字节；无生产补丁。两次预演的closure凭据能校验，但capture接受仍要求独立几何world factory；同一步重复观测、guard返回与几何证书是历史API输入。原始六次预演/两条正式链的分母不变，不能据此计新的物理成功。详见 `provenance/reviews/capture019-preview002-review.json`。父端汇总曾误把整数groups当列表，保留错误后读取既有结果完成汇总，未重跑测试。

15物体复盘已父端复核199项交付与29份运行源文件，9组证据语义/48个合成公式边界检查通过；不等于原始闭环复现。报告仅保留capture校验布尔值，无法证明021实际激活次数与目标变化，提出的报告补丁尚未采用。两次只读采集先后超过整体及单run的8MiB压缩上限，均保留失败且没有完整交付；既定采集计划停止，没有物理重试。22号机正在准备有明确边界的流式原始证据传输候选，须父端审核后执行。详见 `n15-feedback-postmortem-review.json` 和 `n15-capture-transfer-gap.json`。

经验交付也发现实际覆盖缺口：默认600和指定signature的1000 token摘要都漏掉本次必读记录，远端因缺少精确版本停在封包前。父端显式提供五条经链/对象核验的原记录后完成交付；这不能算自动检索成功。缺口已作为独立未解决工程问题追加，当前观察器依赖代码未变。
