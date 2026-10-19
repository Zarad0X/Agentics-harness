# 安装、复现与恢复

parent先复制统一格式并确认AST一致的活动core、指定11个adapters、2个transport和两份原始测试，生成experience/issues.json、regressions.json及活动清单。不要把旧全树交付展开回主树。

## 活动源清单

`provenance/ACTIVE_SOURCE_MANIFEST.json`的schema为`harness-active-source/1`，files为repo相对路径到`{bytes,sha256,source_path,role}`的映射。至少包括：

- harness/{evolution.py,parent_adapter.py,usage_accounting.py}
- adapters/{dispatch22_data3.py,collect_live_combined012_15.py,launch_existing_combined15.py,account22.py,append_completed_usage.py,completed_usage_source.py,sync_completed_usage22.py,watch019020_terminals.py,watch_n15_feedbackpair_001.py,upload_reference22.py,upload_evidence22_data3.py}
- transport/{codex22.py,remote15.py}
- tests/core/{test_evolution.py,test_revision.py}
- experience/{issues.json,regressions.json}

校验要求清单恰好覆盖这几个活动目录的全部文件，不含根docs/scripts/CI；大小与hash由parent读取实际源测量。source_path仅是来源文字，工具不会据此打开旧工作区。新活动文件应在审查提交中同步加入清单，旧文件不得伪称当前源。经验摘要保留真实raw issue status、unresolved与source refs；由parent记录摘要到原快照/行/对象的映射，本工具校验字节，不替代派生语义审查。

## 历史包与显式下载

主树`archive/index.json`已经写入用户提供的实测元数据，不是本任务重新测量：private `Zarad0X/robotwin-astra-harness`，tag `history-20261019`，asset `harness-baseline-20261019.tar.gz`。SHA-256为`69de27a52df93cf02abc528474b6be28fb65d98cc0230e6ca0ac2358b81f3a16`；原SOURCE_MANIFEST摘要为`69ee1e79be4416359d6771e92ff21d51f6bd91e71d0b6be5c62d1e061aef5bcd`。包23,341,870 bytes，展开101,996,987 bytes/6388普通文件；parent发布前独立比对。

已有GitHub CLI认证、可读私有Release后执行README的下载命令。工具不运行gh auth、不读取/输出token、不复制官方ChatGPT session；两种账户彼此独立。下载失败时原缓存不变，partial download-*留在.local供检查。已存在坏缓存会报错，由使用者明确移走/改名后重试；工具不静默clobber。不要将缓存或恢复目录强制git add。

parent的初次本地检查完全无需联网：

```sh
python3 -B scripts/check_repository.py --archive /实际本地/harness-baseline-20261019.tar.gz
python3 -B scripts/test_core.py --archive /实际本地/harness-baseline-20261019.tar.gz
```

本任务不提供也不下载该真实包。最终parent应审查全部可执行文件/workflow，针对实际current core和实际包运行相同检查。CI显式第一步用限时GH_TOKEN下载，后续离线；private Release未就绪时CI应该失败，不跳过归档检查或44项测试。

## 原包内部约定

支持根相对文件名，或所有文件位于同一目录前缀；原`provenance/SOURCE_MANIFEST.json`必须唯一，schema为`harness-source-snapshot/1`。其中files保持原导出格式，包内所有源文件按bytes/hash校验。snapshot index位于`evidence/ledger-snapshots/index.json`，形如`{"snapshots":[{"path":"evidence/ledger-snapshots/<实际ID>"}]}`，也可用路径字符串。每个快照含ledger.jsonl、HEAD.json、objects/<hash>及可选LOCK；目录自身可不作为tar成员出现。检查器支持所有显式追加的快照，不硬编码131行或单快照。

历史核心路径为`work/astra-roundtrip/{evolution.py,parent_adapter.py,usage_accounting.py}`。冻结测试在`tests/core/`；fixtures在`tests/fixtures/`，包含PINS.json、previous-evolution.py、incoming/*和adapters/下的五个wrapper：append_completed_usage、completed_usage_source、dispatch22_data3、collect_live_combined012_15、launch_existing_combined15（均.py.patched）。原测试未改写，通过临时layout和m.IN适配。若实际包内部API/路径不符，报告并修工具，不伪造测试或静默使用旧core。

## 查看旧版本、恢复可编辑历史副本

先运行完整check_repository。下面第一条只列出包中匹配路径；将占位ID改为实际index中的ID。选择恢复会去掉prefix，目标必须是不存在的.local子目录：

```sh
python3 -B scripts/restore_archive.py --prefix evidence/ledger-snapshots
SNAPSHOT_ID='替换成实际快照ID'
python3 -B scripts/restore_archive.py --prefix "evidence/ledger-snapshots/$SNAPSHOT_ID" --destination .local/evolution-ledger
python3 -B harness/evolution.py --store .local/evolution-ledger verify
python3 -B harness/evolution.py --store .local/evolution-ledger brief --stage capture --signature capture_drive_contract
```

先把SNAPSHOT_ID替换为实际值。恢复文件采用新普通文件的可写权限，不沿用归档只读mode；不修改包、不修HEAD。仅用于历史检索和可丢弃实验，不当作当前live authority。查看某个旧runtime可用`--prefix work/astra-roundtrip/v36-live-combined-020/runtime --destination .local/inspect-runtime020`；恢复不执行任何内容。大文件只在忽略目录。保持tar原字节和RUNTIME_BINDING身份，不因可移植性改写历史路径。

## 操作集成与开发验证

```sh
python3 -B scripts/stage_operations.py
PYTHONPATH=harness python3 -c 'from evolution import Store; print(Store.__name__)'
python3 -B scripts/check_code.py
```

stage遇目标冲突即失败，产物不是ready-to-run部署。parent需明确配置外部权威ledger、兼容source/config及已配置splitVPN/SSH，另行审查硬编码路径；不能从恢复样本推导live配置。当前生产协调器仍在原工作区。本次没有remote部署命令。

shared server22测试使用parent认可的idle CPU lease与≤30CPU/60wall每子任务、≤180累计CPU的本次预算；不在工具内偷偷选择GPU/模型。失败临时验证目录保留于.local/validation，运行日志应由调用方保存。Python使用POSIX flock/fork，Windows不支持；macOS已由parent独立通过67项测试。科学core和旧源码不做lint/format扫描；`check_code.py`只编译新scripts/tests/scaffold并检查空白，不执行编译内容，不产生pycache。
