# 从成功基线出发迭代

用户于2026-10-19明确要求采用以下顺序，完整往返目标不变。

1. 核验并原样复现ten-r21：原始seed2、10物体、原辅助条件和搜索策略，先抓出并放到原任务槽位。历史成功证据不丢弃，也不代替本次复现。
2. 在已验证父版本上逐项增加能力。优先保留已成功的抓取，再增加全部取出后恢复各自原位置与原姿态；随后验证15/20物体和多个布局。
3. 每个候选明确父版本、最小源码/配置差异、同条件端到端回归及全部尝试。保住父版本已验证能力，并通过新增能力的独立验收后才能晋升。
4. 发生退化时保留证据、隔离差异，回到最近已验证父版本继续；不叠加尚未验证的修改。原始失败、反例和推翻的假设不删除。

组件测试、CPU诊断、更多日志和更严格保护都不等于任务成功。单次10/10复现不证明稳定成功率、多布局泛化、纯摩擦或真实机器人效果。历史单向验收与新严格往返的接触、速度、安全和RETURN验收分别报告；不能为维持旧分数而放宽新标准。

原工作区的不可变基线为`work/astra-direct/verified-ten-r21`，身份与历史结果见`BEST_VERIFIED.json`。最近父端核对11份算法、114份依赖源码/机器人文件与107份资产均匹配原哈希。67号机的旧assets链接指向系统盘；先准备数据盘上的独立精确依赖副本，保留共享链接与原件，不复制整个共享资产树。

22号机官方ChatGPT账号的gpt-6-astra已开始`restore-r21-baseline-001`，准备依赖、受控复跑与独立验收入口。当前仍是工程准备，不是已经重新跑出10/10。拟定首轮只有一条、最长7200秒、无自动重试，父端先审查交付并重新检查资源。原024和现有观察器保留既定预算与归属，不因主线调整而打断或重复运行。

本仓库`controller/`暂保留024的已审查封存源码，作为诊断证据。Git提交不是部署；后续基线恢复交付经审核后，再单独决定维护入口与部署变化。原工作区`BASELINE_FIRST.json`及持续监控保存当前执行优先级，进程真相仍以原始运行和审计证据为准。

## 2026-10-19 parent recovery review

The first recovery packet passed 18 producer synthetic tests and all 26 manifest entries were independently rehashed. It is **not launchable on actual67**: it couples native Python to pidfd support and requires unavailable bwrap/writable cgroup. A narrow server22 Astra repair is in progress; no baseline physics attempt has been launched. See `provenance/reviews/r21-recovery001-final-review.json` and `r21-actual67-launch-constraints.json`.

All ten fixtures must come from immutable server67 r21. Two current loose local fixtures differ (convex meshes and robot vertices); this is a fidelity finding, not a demonstrated cause of later physical failures. The 243-file closure inventory totals 51,038,200 bytes. The 106 GLBs have no external URI references and ten collision-directory inventories match; the static scanner also reports non-runtime XML references requiring source-bound classification. Closure copy and dynamic environment review remain pending.

## 2026-10-20 actual67 baseline attempt

Recovery002 passed all 32 independent actual67 CPU regressions. A new regular-file data-disk closure preserved all 11 algorithms, 114 source files, 107 selected assets and ten fixtures. The original shared assets link was untouched. Native and supervisor interpreters were reviewed separately; selected native library hashes cover 88,323,751 bytes, with incomplete historical environment coverage explicitly retained.

`ten-r21-rebaseline-20261020-001` started after fresh GPU4/core/job admission and exited 1 during `import envs`, before scene construction or grasp. The imported clutter helper unconditionally reads Objaverse metadata, even with clutter disabled. Native wall time was 9.517536 s and wait4 child CPU 7.808001 s; no surviving owned processes or accounting gaps were recorded. Independent acceptance rejected the failed terminal. OUT/RETURN counts are unavailable because no task terminal state exists; this is neither a new 10/10 nor a completed physical episode. The one-attempt protocol is spent and will not auto-retry.

A data-only correction for the two confirmed missing metadata JSON files (28,396 bytes) and a bounded import-only regression is being prepared on server22. Another physical attempt needs new source/dependency review, an explicit finite protocol and fresh admission after that regression. Source packet and exact test evidence are in the private `r21-recovery002-20261020` prerelease; see `provenance/reviews/r21-recovery002-private-archive.json`. Existing024 independently remains with its original audit owner.

## Actual native import readiness and baseline002

The exact two missing metadata JSONs are present in new data closure004 (245files,51,066,596bytes); all original243 entries retain their hashes. Parent actual67 passed19/26/30 scoped regressions. The final guarded native import passed in17.473510wall/16.911553childCPU with no scene constructed and no surviving processes/accounting gaps. Earlier null-device and constructor-type diagnostic failures remain retained; they did not modify original r21 code.

A NEW single-attempt protocol002 was then reviewed and started, with the same7200wall/childCPU caps and no automatic retry. The parent driver requires the exact successful import receipt, closure and probe/source bindings before admitting physics, and owns one independent terminal audit. Initial-state, grasp-preview and placement/release events are progressing; accepted counts and10/10 reproduction remain pending terminal verification. A one-way extraction result will not count as original-pose RETURN or stable multiseed/multilayout success.

Exact diagnostic packets003–005 and parent launch gate sources001/002 are private release attachments indexed in `provenance/reviews/r21-import-repair-private-archives.json`. New baseline IDs currently use a separate parent all-attempt index because the core rt-v36 parser has not yet been extended; no unified scorecard coverage is claimed. No active observer/runtime dependency was edited for publication.

## Recorded initial-state parity for baseline002

The server22 Astra comparison and independent parent reproduction found exact equality of all146 recorded physical scalars between historical successful r21 and baseline002: ten object poses/extents/top heights and both arms q/qdot. The initial events differ only in time_unix. Whole-mesh size/digest and all ten per-object digest records also match. Nine focused math/key checks pass, and the complete derived summary reproduces exactly.

This rules out a difference in these recorded initial fields; object velocities, commanded home/control targets, internal solver/contact/RNG state and historical environment attestation are absent. Neither matching source/initial snapshots nor differing timestamped event hashes establish later dynamics or OUT/RETURN success. Baseline002 remains a bounded original one-way reproduction with its existing driver-owned terminal audit. See the derived review and private source archive in `provenance/reviews/r21-initial-fidelity001-review.json`; raw state input and CLI logs are excluded. This is no runtime deployment or stable-version promotion.
