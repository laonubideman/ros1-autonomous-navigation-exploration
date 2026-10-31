#!/usr/bin/env bash
# 只停止本工作空间启动脚本记录的基础导航进程，不使用全局 killall/pkill。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
python3 - <<'PY'
from pathlib import Path
import os
import signal
import time
pid_file = Path('logs/launch.pid')
if not pid_file.exists():
    raise SystemExit('没有基础导航启动记录；直接 roslaunch 的终端请使用 Ctrl+C。')
pid = pid_file.read_text().strip()
if not pid.isdecimal():
    raise SystemExit('PID 记录格式不正确。')
command = Path('/proc')/pid/'cmdline'

# 空 cmdline 表示已退出的僵尸进程，不再把它当成正在运行的 launch。
def running():
    try:
        return bool(command.read_bytes())
    except FileNotFoundError:
        return False

if running():
    arguments = command.read_bytes().decode().split('\0')
    allowed = ('exam_fastlio' in arguments and 'fastlio_navigation.launch' in arguments) or \
              ('exam_bringup' in arguments and 'navigation.launch' in arguments)
    if not allowed:
        raise SystemExit('记录的进程不是本例程基础导航，拒绝停止。')
    os.kill(int(pid),signal.SIGINT)
    deadline = time.monotonic()+30
    while running() and time.monotonic()<deadline:
        time.sleep(0.1)
    if running():
        raise SystemExit('基础导航仍在退出，请查看启动终端。')
pid_file.unlink()
PY
