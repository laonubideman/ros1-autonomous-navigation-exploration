#!/usr/bin/env bash
# 只停止本脚本记录的探索 launch；不停止导航、建图与定位。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
# 以下内嵌 Python 执行进程检查、数据记录或部署核对。
python3 - <<'PY'
from pathlib import Path
import os, signal, time
pid_file = Path('logs/exploration.pid')
if not pid_file.exists():
    raise SystemExit('No exploration launch PID recorded.')
text = pid_file.read_text().strip()
if not text.isdecimal():
    raise SystemExit('Invalid exploration launch PID.')
cmdline = Path('/proc') / text / 'cmdline'
if cmdline.exists():
    args = cmdline.read_bytes().decode().split('\0')
    if 'exam_fastlio' not in args or 'exploration.launch' not in args:
        raise SystemExit('Recorded process is not the exploration launch.')
    os.kill(int(text), signal.SIGINT)
    deadline = time.monotonic()+30
    def running():
        try:
            return bool(cmdline.read_bytes())
        except FileNotFoundError:
            return False
    while running() and time.monotonic()<deadline:
        time.sleep(0.1)
    if running():
        raise SystemExit('Exploration launcher still shutting down.')
pid_file.unlink()
PY
