#!/usr/bin/env bash
# 在已运行的导航上独立开启探索；记录本次 launch PID，避免重复启动。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
# 加载 ROS 或当前工作空间环境，确保能找到功能包与消息类型。
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
# 以下内嵌 Python 执行进程检查、数据记录或部署核对。
python3 - <<'PY'
import socket, rosnode
socket.setdefaulttimeout(2)
try:
    nodes = rosnode.get_node_names()
except rosnode.ROSNodeIOException:
    nodes = []
if '/move_base' not in nodes or '/fastlio_grid' not in nodes:
    raise SystemExit('Start the FAST-LIO2 navigation demo first.')
if '/frontier_explorer' in nodes:
    raise SystemExit('Autonomous exploration is already running.')
PY
mkdir -p logs
printf '%s\n' "$$" > logs/exploration.pid
# 用目标进程替换脚本进程，保留 Ctrl+C 和记录 PID 的对应关系。
exec roslaunch exam_fastlio exploration.launch "$@"
