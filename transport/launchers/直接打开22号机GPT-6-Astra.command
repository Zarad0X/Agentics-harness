#!/bin/bash
set -euo pipefail

# Opens Codex directly on SERVER22 in the GPT-6 boundary project.
# The SSH host alias uses the existing VPN tunnel and SERVER22's own Codex login.
/usr/bin/python3 "$(cd "$(dirname "$0")" && pwd)/codex22.py" prepare
exec /usr/bin/ssh -tt \
  -F "$HOME/.ssh/config" \
  -o BatchMode=yes \
  -o ClearAllForwardings=yes \
  -o ConnectTimeout=20 \
  -o StrictHostKeyChecking=yes \
  22-codex \
  'export TERM=xterm-256color; exec codex -C /data4/yuantao/astra-roundtrip-20261009'
