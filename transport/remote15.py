"""Local SSH transport; keys stay on this Mac."""

import subprocess, sys, shlex, contextlib
from pathlib import Path

recovery_marker = (
    Path(__file__).resolve().parents[1] / "astra-roundtrip/NETWORK_RECOVERY_PENDING.json"
)
if recovery_marker.exists():
    print(
        "Remote observation suspended pending connection recovery; no VPN startup or remote command attempted. See "
        + str(recovery_marker),
        file=sys.stderr,
    )
    sys.exit(78)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "outputs"))
import codex22

args = [
    "ssh",
    "-o",
    "ClearAllForwardings=yes",
    "-o",
    "BatchMode=yes",
    "-o",
    "ConnectTimeout=60",
    "-o",
    "ServerAliveInterval=10",
    "-o",
    "ServerAliveCountMax=3",
]
if not codex22.split_route_present("10.20.3.15"):
    with contextlib.redirect_stdout(sys.stderr):
        codex22.ensure_proxy()
    base = [v.replace("ConnectTimeout=10", "ConnectTimeout=45") for v in codex22.ssh_base()]
    proxy = shlex.join(base[:-1] + ["-W", "10.20.3.15:22", base[-1]])
    args += ["-o", "ProxyCommand=" + proxy]
args += ["A6000-15"]
command = sys.argv[1:] or ["python3", "-"]
r = subprocess.run(args + [shlex.join(command)], input=sys.stdin.read(), text=True)
sys.exit(r.returncode)
