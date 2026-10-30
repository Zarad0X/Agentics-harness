# SERVER22 orphan proxy reconnect repair

The interactive launcher repeatedly exited because localhost19062 was retained by an
ocproxy whose VPN parent had exited. Live inspection found no OpenConnect process,
no SERVER22 split route, a failed direct SSH connection, and a failed forwarded SSH
handshake. PID7285 belonged to the current user, had PPID1, and matched the dedicated
fixed-target proxy command exactly. Its start time was October22.

The previous handler correctly rejected an unusable listener but offered no recovery.
The new handler revalidates process identity, owner, start time, parent and command,
checks that no OpenConnect process exists, then sends SIGTERM only to that orphan.
It waits for the port to close before using the original external VPN provisioning
script. Unknown ownership, process-inspection errors, a live VPN, a remaining listener
or a retained split route block creation of another VPN. No sudo action is needed
for this user-mode transport. No unrelated process is terminated.

Authenticated direct connectivity is now probed independently of the exact route
hint. All three interactive launchers run `prepare` before starting SSH, so tunnel
startup is outside the interactive SSH handshake and failures have a direct message.

Validation: 17 focused transport regressions passed for both runtime and maintained
copies; three launchers passed Bash syntax checks. Actual recovery removed the
identified orphan and started the original dedicated VPN. The same22-codex alias
returned G4000A100/yuantao and survived35seconds idle. All runtime function ASTs match
the maintained source. The first maintained test invocation exposed a fixture that
depended on the runtime-only provisioning path; that failure is retained in the
receipt, and the fixture now supplies an isolated dummy script.

Evidence: external `work/transport22-repair-20261030/REPAIR_RECEIPT.json`, source review
`provenance/reviews/20261030-codex22-orphan-recovery.json`, and engineering experience
`transport22-orphan-proxy-20261030@1` in the authoritative external evolution ledger.
The transport-stage engineering brief had no matching issue before this repair.

The cause of the original VPN process exit remains unknown. A35second check is not
long-term availability proof. No Codex model request, quota check, remote simulation
or physical robotics experiment ran. Existing unrelated untracked adapter data and
frozen historical sources were preserved. Launchers retain their original external
workspace paths and require existing local SSH/VPN provisioning; copying this repo
does not provision or deploy those dependencies.
