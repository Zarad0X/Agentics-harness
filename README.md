# RoboTwin Astra Harness

密集双臂抓取与原位放回的研发 Harness，私有仓库 `Zarad0X/robotwin-astra-harness`。主树保留当前核心、适配器、单份开发控制器、测试和文档；历史脚本、22个旧版本及完整证据集中归档，按需恢复。

**完整10/15/20物体往返验收尚未完成。** 当前 `controller/` 是源码和CPU复核后的024开发候选，物理效果尚待验证。ten-r21 assisted10/10单向基线现已原样复现并通过独立验收；严格原位RETURN尚未完成。见 [状态与边界](docs/STATUS.md)；本仓库不是实时进程监控面板。

**当前研发优先级已改为从ten-r21成功基线出发。** 先原样复现，再做最小改动、同条件端到端对照，保住父版本能力后逐项扩展。024继续作为既有诊断候选留证，不默认替代成功基线；其目录尚未因这项计划调整而部署或替换。见[基线优先迭代](docs/BASELINE_FIRST.md)。

2026-10-20用户进一步明确：原样单向10/10已独立复现，当前优先交付是一条完整抓出并恢复原位置、原姿态的GPU实验。暂停扩大026–031外围资格测试；只检查直接启动、清理与RETURN接线改动。034已完成真实实验：独立OUT10/10、严格RETURN0/10，放回12号时触发与17号的接触保护；036已完成并独立验收OUT10/10、严格RETURN1/10，完整动作预演挡住了不安全的12号放回，17号最终稳定。037正通过22号机官方ChatGPT账号准备安全RETURN路径修复；当前没有新仿真，完整往返仍未通过。

## 主树结构

| 目录 | 只放什么 |
|---|---|
| `harness/` | 当前三个核心模块，统一格式，保持原接口及逻辑 |
| `controller/` | 单份024开发控制器，保持封存源码及绑定；尚未物理验收 |
| `adapters/` | 10个活跃父控制器集成源，保留历史主机路径语义 |
| `transport/` | `codex22.py`、`remote15.py`，需要已配置环境 |
| `scripts/` | 维护中的下载、离线校验、44项测试、选择恢复和目录准备工具 |
| `tests/core/` | 两份未修改的原始44项测试模块；fixtures不在主树 |
| `tests/scaffold/` | 新工具的合成/对抗回归，不能计为物理结果 |
| `experience/` | `LATEST.json`指向经父端审核的经验快照，保留issue/registry及证据引用 |
| `provenance/` | 当前活动源的 `ACTIVE_SOURCE_MANIFEST.json` |
| `archive/index.json` | 私有Release资产的大小、摘要、成员数与原SOURCE_MANIFEST摘要 |
| `.local/` | 忽略的下载缓存、临时验证、可丢弃恢复和操作布局 |

本脚手架不携带实际活动源、经验摘要或历史包。parent负责从真实本地源复制活动文件、生成清单、发布私有Release并独立复核；缺少这些内容时工具必须失败。

## 第一次运行

Linux/macOS上的Python3.8+，本地POSIX文件系统。核心和维护工具只用标准库。完整验证还需要已认证、可读此私有仓库Release的GitHub CLI；认证由使用者单独配置，GitHub凭据与server22官方ChatGPT模型账号无关。本次未实测macOS。

```sh
mkdir -p .local/archive-cache
python3 -B scripts/fetch_archive.py
python3 -B scripts/check_code.py
python3 -B scripts/check_controller.py
python3 -B scripts/check_repository.py
python3 -B -m unittest discover -s tests/scaffold -v
python3 -B scripts/test_core.py
```

下载是唯一显式联网步骤。已有缓存仍要校验，不覆盖已有坏文件；缺失或有symlink的缓存父目录会被拒绝。校验/测试不会偷偷fetch。parent初次离线检查可分别为 `check_repository.py` 和 `test_core.py` 提供 `--archive /实际本地/harness-baseline-20261019.tar.gz`，无需先发布或联网。

历史资产位于私有Release `history-20261019`，23,341,870 bytes，展开101,996,987 bytes、6388个普通文件（数字与hash来自parent提供的测量，parent已独立校验实际历史包）。包不进入Git或LFS；正常clone不携带23 MiB二进制或约100 MiB展开历史。完整校验流式检查全包，再仅恢复账本闭包到临时目录调用原 `Store.rows()`。44项测试只恢复必需fixtures，以**当前 `harness/` 核心**运行；历史核心不会替代当前核心。

## 独立核心与操作适配器

核心导入使用 `PYTHONPATH=harness`，无需改成package。恢复一份可丢弃旧账本后：

```sh
python3 -B harness/evolution.py --store .local/evolution-ledger verify
python3 -B harness/evolution.py --store .local/evolution-ledger brief --stage capture --signature capture_drive_contract
```

恢复步骤见 [复现文档](docs/reproduction.md)。旧账本是历史副本，不是活的权威来源。`scripts/stage_operations.py` 可建立新的 `.local/operations/{outputs,work/astra-direct,work/astra-roundtrip}`，只复制精确映射的源文件，遇已有目标即失败。它不配置SSH/VPN、账本、源码配置，不改绝对路径，也不执行脚本。原工作区仍是当前生产协调器；实际接入前必须由parent明确配置外部权威ledger和兼容source/config，并审查环境绑定。

## 修改与验证

先读 [AGENTS](AGENTS.md)、[架构](docs/architecture.md) 和 [迭代规则](docs/iteration.md)。新工具遵循清晰的标准Python风格；`pyproject.toml`只为新工具/新测试配置lint/format，不对成熟科学核心、适配器或原测试做外观整理。标准库CI执行源码编译/空白检查，不安装依赖。

每次修复使用conventional commit，保留症状、假设、确认原因、全部尝试、回归和独立复核。代码、真实物理、多布局、heldout、采纳门槛分别报告。新经验先追加到外部当前权威Store，再形成不同身份的审查快照/归档，初始tar永不修改；当前清单在明确审查后更新，不永久阻止正常源代码演进。

共享server22测试先取得sampled-idle cooperative CPU lease，单核单线程，每子任务≤30 CPU秒/60 wall秒，本次授权批次累计≤180 CPU秒。GitHub Actions只在显式下载后做离线检查，不自动运行observer/SSH/native/model。Git历史从导出开始；更早记录保留在历史包。私有初始导出不添加开源许可证。
