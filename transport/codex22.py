#!/usr/bin/env python3
"""Send a task to the ChatGPT-authenticated Codex CLI on the VPN-only A100 host.

The prompt is read from stdin. SSH uses a dedicated key; no SSH password or
ChatGPT token is copied into this workspace.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone


HOST = "10.20.3.22"
USER = "yuantao"
PORT = "9062"
KEY = Path.home() / ".ssh/id_ed25519_codex_a100_22"
REMOTE_CODEX = "/home/yuantao/.nvm/versions/node/v24.19.0/bin/codex"
PROJECT_ROOT = Path(__file__).resolve().parent.parent
STATE_FILE = PROJECT_ROOT / "work/remote-codex22-sessions.json"
PROXY_SCRIPT = PROJECT_ROOT / "outputs/vpn-22-user-proxy.sh"
PROXY_LOG = PROJECT_ROOT / "work/vpn-22-user-proxy.log"
PROXY_LOCK = PROJECT_ROOT / "work/vpn-22-user-proxy.lock"
LOCAL_PORT = int(os.environ.get("CODEX22_LOCAL_PORT", "19062"))
if LOCAL_PORT not in (19062, 19063):
    raise ValueError("Unsupported project VPN proxy port")
UUID = re.compile(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}\Z")
_DIRECT_SSH = False


def split_route_present(host):
    """Recognize an existing exact macOS VPN host route without changing routes."""
    if sys.platform != "darwin":
        return False
    result = subprocess.run(
        ["/sbin/route", "-n", "get", host], capture_output=True, text=True, timeout=3
    )
    fields = dict(re.findall(r"^\s*([\w ]+):\s*(.*?)\s*$", result.stdout, re.M))
    return (
        result.returncode == 0
        and fields.get("destination") == host
        and fields.get("interface", "").startswith("utun")
        and (
            fields.get("mask") == "255.255.255.255"
            or "HOST" in fields.get("flags", "").strip("<>").split(",")
        )
    )


def fail(message):
    raise SystemExit(message)


def proxy_listening():
    try:
        with socket.create_connection(("127.0.0.1", LOCAL_PORT), timeout=1):
            return True
    except OSError:
        return False


def ssh_ready(direct):
    """Confirm the selected transport reaches this server with our SSH key."""
    try:
        result = subprocess.run(
            ssh_base(direct=direct) + ["true"],
            stdin=subprocess.DEVNULL,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except subprocess.TimeoutExpired:
        return False
    return result.returncode == 0


def stop_proxy_group(process, grace_s=5):
    """Stop our newly created session even if its shell exits before VPN children."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + grace_s
    while time.monotonic() < deadline:
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.1)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait(timeout=2)


def ensure_proxy():
    global _DIRECT_SSH
    # Reuse the user's existing split tunnel. A second VPN login can interfere
    # with it. A route or a listening port alone does not prove a usable tunnel.
    _DIRECT_SSH = False
    if split_route_present(HOST):
        if ssh_ready(direct=True):
            _DIRECT_SSH = True
            return
        if proxy_listening() and ssh_ready(direct=False):
            print(
                "分流直连暂不可用，已改用经 SSH 验证的现有专用通道。",
                file=sys.stderr,
                flush=True,
            )
            return
        fail(
            "服务器 VPN 路由仍存在，但直连和现有专用通道均无法完成 SSH 握手。"
            "请重连现有三台服务器 VPN；未启动重复 VPN。"
        )
    PROXY_LOCK.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(PROXY_LOCK, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX)
        if proxy_listening():
            if ssh_ready(direct=False):
                return
            fail(
                "本机专用通道端口仍在监听，但无法完成服务器 SSH 握手。"
                "请恢复现有 VPN 通道；未启动重复 VPN。"
            )
        if LOCAL_PORT != 19062:
            fail("指定的独立 TCP VPN 通道未监听；不改动或重启其他通道。")
        if not PROXY_SCRIPT.is_file():
            fail(f"用户态 VPN 启动脚本不存在：{PROXY_SCRIPT}")
        log_fd = os.open(PROXY_LOG, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
        with os.fdopen(log_fd, "a", encoding="utf-8") as log:
            print("正在建立 22 服务器专用 VPN 通道…", file=sys.stderr, flush=True)
            proxy_env = {
                "PATH": "/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin",
                "HOME": str(Path.home()),
                "USER": os.environ.get("USER", "zhanghan"),
                "LOGNAME": os.environ.get("LOGNAME", "zhanghan"),
                "TMPDIR": os.environ.get("TMPDIR", "/private/tmp"),
            }
            process = subprocess.Popen(
                ["/bin/bash", str(PROXY_SCRIPT)],
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=proxy_env,
            )
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if proxy_listening():
                break
            if process.poll() is not None:
                stop_proxy_group(process)
                fail(f"用户态 VPN 未启动；查看 {PROXY_LOG}。")
            time.sleep(0.25)
        if proxy_listening():
            if ssh_ready(direct=False):
                return
            stop_proxy_group(process)
            fail("新专用通道未能完成服务器 SSH 握手；已停止本次创建的通道。")
        # Only this invocation's fresh process group is eligible for cleanup.
        # A timed-out startup must not accumulate another reconnecting VPN.
        stop_proxy_group(process)
        fail(f"等待用户态 VPN 超时；查看 {PROXY_LOG}。")
    finally:
        fcntl.flock(fd, fcntl.LOCK_UN)
        os.close(fd)


def ssh_base(direct=None):
    if not KEY.is_file():
        fail(f"专用 SSH 密钥不存在：{KEY}")
    direct = _DIRECT_SSH if direct is None else direct
    return [
        "/usr/bin/ssh",
        "-T",
        "-F",
        "/dev/null",
        "-i",
        str(KEY),
        "-o",
        "IdentitiesOnly=yes",
        "-o",
        "BatchMode=yes",
        "-o",
        "StrictHostKeyChecking=yes",
        "-o",
        "ConnectTimeout=10",
        "-o",
        "ConnectionAttempts=1",
        "-o",
        "ClearAllForwardings=yes",
        "-o",
        "ServerAliveInterval=30",
        "-o",
        "ServerAliveCountMax=3",
        "-o",
        "TCPKeepAlive=yes",
        "-o",
        f"HostKeyAlias=[{HOST}]:{PORT}",
        "-o",
        "LogLevel=ERROR",
        "-p",
        PORT if direct else str(LOCAL_PORT),
        f"{USER}@{HOST if direct else '127.0.0.1'}",
    ]


def remote_command(args):
    return 'export PATH="$HOME/.nvm/versions/node/v24.19.0/bin:$PATH"; exec ' + shlex.join(args)


def load_sessions():
    if not STATE_FILE.exists():
        return {}
    return json.loads(STATE_FILE.read_text(encoding="utf-8"))


def save_sessions(sessions):
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    temp = STATE_FILE.with_suffix(".tmp")
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        json.dump(sessions, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    os.replace(temp, STATE_FILE)


def run_exec(remote_args, prompt, session_info=None):
    proc = subprocess.Popen(
        ssh_base() + [remote_command(remote_args)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=sys.stderr,
        text=True,
        encoding="utf-8",
        bufsize=1,
    )
    proc.stdin.write(prompt)
    proc.stdin.close()
    session_id = None
    final_message = None
    turn_completed = False
    for line in proc.stdout:
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            print(line.rstrip(), file=sys.stderr)
            continue
        kind = event.get("type")
        if kind == "thread.started":
            session_id = event.get("thread_id")
            if session_info is not None and session_id:
                sessions = load_sessions()
                sessions[session_id] = session_info
                save_sessions(sessions)
            print(f"REMOTE_SESSION_ID={session_id}", file=sys.stderr, flush=True)
        elif kind == "item.completed":
            item = event.get("item") or {}
            if item.get("type") == "agent_message":
                final_message = item.get("text")
        elif kind == "turn.completed":
            turn_completed = True
        elif kind == "turn.failed":
            print(json.dumps(event, ensure_ascii=False), file=sys.stderr)
    code = proc.wait()
    if final_message:
        print(final_message)
    if code or not turn_completed:
        fail(
            f"远端 Codex 未确认完成（SSH 退出码 {code}，会话 {session_id or '未取得'}）。"
            "先检查该会话状态，不要盲目重试。"
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="检查免密码 SSH 和远端 Codex 登录")
    sub.add_parser("connect", help="供 SSH ProxyCommand 使用的已验证原始连接")
    run = sub.add_parser("run", help="发起一个新的远端 Codex 会话，指令从 stdin 读取")
    run.add_argument("--cwd", default="/home/yuantao", help="远端工作目录")
    run.add_argument("--write", action="store_true", help="允许 Codex 写入指定工作目录")
    run.add_argument("--model", help="可选的远端模型覆盖")
    resume = sub.add_parser("resume", help="续接由本脚本创建的指定会话")
    resume.add_argument("session_id", help="远端 Codex 会话 UUID")
    args = parser.parse_args()
    ensure_proxy()
    if args.command == "connect":
        endpoint = HOST if _DIRECT_SSH else "127.0.0.1"
        port = PORT if _DIRECT_SSH else str(LOCAL_PORT)
        # Do not add nc -w: its idle timeout would terminate interactive SSH.
        os.execv("/usr/bin/nc", ["nc", endpoint, port])
        return
    if args.command == "check":
        result = subprocess.run(
            ssh_base() + [remote_command([REMOTE_CODEX, "login", "status"])],
            stdin=subprocess.DEVNULL,
            check=False,
        )
        raise SystemExit(result.returncode)
    prompt = sys.stdin.read()
    if not prompt.strip():
        fail("指令为空；请通过 stdin 提供任务内容。")
    if args.command == "run":
        if not args.cwd.startswith("/") or args.cwd == "/":
            fail("--cwd 必须是远端的具体绝对路径。")
        if args.write and args.cwd == "/home/yuantao":
            fail("写入任务请指定具体项目目录，避免对整个远端主目录开放写入。")
        sandbox = "workspace-write" if args.write else "read-only"
        remote_args = [
            REMOTE_CODEX,
            "exec",
            "--json",
            "--skip-git-repo-check",
            "--sandbox",
            sandbox,
            "-C",
            args.cwd,
        ]
        if args.model:
            remote_args += ["--model", args.model]
        remote_args += ["-"]
        info = {
            "cwd": args.cwd,
            "sandbox": sandbox,
            "model": args.model,
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        run_exec(remote_args, prompt, info)
    else:
        if not UUID.fullmatch(args.session_id):
            fail("会话 ID 格式不正确。")
        sessions = load_sessions()
        info = sessions.get(args.session_id)
        if info is None:
            fail("该会话不是由本脚本建立；停止以避免续接其他人的任务。")
        remote_args = [
            REMOTE_CODEX,
            "exec",
            "-C",
            info["cwd"],
            "resume",
            "--json",
            "--skip-git-repo-check",
            "-c",
            f'sandbox_mode="{info["sandbox"]}"',
            args.session_id,
            "-",
        ]
        if info.get("model"):
            remote_args[5:5] = ["--model", info["model"]]
        run_exec(remote_args, prompt)


if __name__ == "__main__":
    main()
