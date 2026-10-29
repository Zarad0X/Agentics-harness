# 2026-10-29 SERVER22 direct connection repair

The interactive SSH connection dropped. Direct VPN SSH then timed out, while the
existing localhost forward still listened but failed the SSH banner exchange. The
existing VPN subsequently reconnected; this repair did not restart or replace it.

The helper now authenticates its selected transport. A failed direct route may use
an existing authenticated forward without creating a duplicate VPN. A stale
listener fails explicitly. The new `connect` command supplies the selected raw
stream to SSH ProxyCommand, eliminating a separate route-only decision in the
local host configuration. SSH keepalive is 30 seconds with three missed replies;
netcat has no idle timeout. This cannot guarantee survival of a real VPN outage.

Eight focused regressions cover direct use, authenticated-forward fallback, stale
routes and listeners, prevention of duplicate VPN creation, probe timeouts and raw
endpoint selection. Local and maintained copies passed. Maintained syntax and
whitespace checks passed. Live SSH returned `G4000A100` / `yuantao`, confirmed the
ChatGPT login method and found the task-context directory. A second connection
survived 35 seconds idle. No model prompt, quota claim, simulation or GPU experiment
was involved.

The runtime workspace retains the live macOS SSH/VPN configuration. Private
configuration and logs are excluded from Git. The older general terminal launcher
now uses the same SSH alias as the project launchers. Existing unrelated untracked
account/delegation/ledger files were preserved and are excluded from this commit;
this is targeted transport validation, not a fresh whole-repository or physics
acceptance result.

Source import review: `provenance/reviews/20261029-codex22-transport.json`. Original
source/archive manifests and controller/algorithm files remain unchanged except
for the explicitly updated current transport entry in ACTIVE_SOURCE_MANIFEST.
