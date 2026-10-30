#!/bin/bash
set -euo pipefail

# Continue the SERVER22 Astra chat that read the screenshot, assets and source.
# All prompts go directly to the remote Codex; its ChatGPT account supplies usage.
/usr/bin/python3 "$(cd "$(dirname "$0")" && pwd)/codex22.py" prepare
exec /usr/bin/ssh -tt \
  -F "$HOME/.ssh/config" \
  -o BatchMode=yes \
  -o ClearAllForwardings=yes \
  -o ConnectTimeout=20 \
  -o StrictHostKeyChecking=yes \
  22-codex \
  'export TERM=xterm-256color; exec codex resume --cd /data4/yuantao/astra-roundtrip-20261009 --sandbox workspace-write --model gpt-6-astra 01a1874d-c45a-7fc0-830b-eaa0e2ee46de'
