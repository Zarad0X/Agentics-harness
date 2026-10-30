#!/bin/bash
set -euo pipefail

printf '%s\n' '正在连接 22 服务器已登录的 Codex…'
/usr/bin/python3 "$(cd "$(dirname "$0")" && pwd)/codex22.py" prepare
exec /usr/bin/ssh -tt \
  -F "$HOME/.ssh/config" \
  -o BatchMode=yes \
  -o ClearAllForwardings=yes \
  -o ConnectTimeout=10 \
  -o StrictHostKeyChecking=yes \
  22-codex \
  'export PATH="$HOME/.nvm/versions/node/v24.19.0/bin:$PATH" TERM=xterm-256color; exec codex -C "$HOME"'
