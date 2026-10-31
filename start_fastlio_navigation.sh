#!/usr/bin/env bash
# 启动 FAST-LIO2 实时建图与手动目标导航；默认不启动自主探索。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
# 加载 ROS 或当前工作空间环境，确保能找到功能包与消息类型。
source /opt/ros/noetic/setup.bash
# 加载当前工作空间环境，确保能找到功能包与消息类型。
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export GAZEBO_MODEL_DATABASE_URI=
# 以下内嵌 Python 执行进程检查、数据记录或部署核对。
python3 - <<'PY'
import socket, xmlrpc.client
socket.setdefaulttimeout(2)
try:
    code, _, _ = xmlrpc.client.ServerProxy('http://localhost:11311').getPid('/fastlio_start')
except (OSError, xmlrpc.client.Error):
    code = 0
if code == 1:
    raise SystemExit('ROS master already running. Stop the active navigation version first.')
PY
mkdir -p logs
printf '%s\n' "$$" > logs/launch.pid
# 用目标进程替换脚本进程，保留 Ctrl+C 和记录 PID 的对应关系。
exec roslaunch exam_fastlio fastlio_navigation.launch "$@"
