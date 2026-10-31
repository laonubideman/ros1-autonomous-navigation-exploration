#!/usr/bin/env bash
# 学习对照入口：使用主办方参考地图和仿真真值里程计，不计正式建图得分。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export GAZEBO_MODEL_DATABASE_URI=
# 与正式导航共用 master 和场景，切换前需停止正在运行的整套导航。
python3 - <<'PY'
import socket
import xmlrpc.client
socket.setdefaulttimeout(2)
try:
    code,_,_ = xmlrpc.client.ServerProxy('http://localhost:11311').getPid('/reference_start')
except (OSError,xmlrpc.client.Error):
    code = 0
if code == 1:
    raise SystemExit('已有 ROS master。请先停止当前导航版本。')
PY
mkdir -p logs
printf '%s\n' "$$" > logs/launch.pid
exec roslaunch exam_bringup navigation.launch "$@"
