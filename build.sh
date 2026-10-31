#!/usr/bin/env bash
# 单工作空间编译入口，适用于原生 Ubuntu 20.04、虚拟机和 WSL2。
# 所有项目源码已随包提供，编译不需要克隆上游仓库，也不需要旧版工作空间。
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/noetic/setup.bash
mkdir -p logs vendor/livox-sdk-install

# 压缩包不依赖外部保留的 src/CMakeLists.txt 符号链接。在全新解压的
# 工作空间中主动初始化 catkin 顶层入口，避免 catkin_make 立即退出。
if [[ ! -e src/CMakeLists.txt && ! -L src/CMakeLists.txt ]]; then
  catkin_init_workspace src
fi

# Windows 解压复制后仍确保 ROS Python 节点可直接执行。
chmod +x ./*.sh src/exam_bringup/scripts/*.py src/exam_fastlio/scripts/*.py
exam_sdk="$PWD/vendor/livox-sdk-install"
cmake -S vendor/Livox-SDK -B vendor/Livox-SDK/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$exam_sdk"
cmake --build vendor/Livox-SDK/build -j2
cmake --install vendor/Livox-SDK/build

# Livox SDK 仅安装在当前工作空间，用实际头文件和静态库完成驱动链接。
export CPLUS_INCLUDE_PATH="$exam_sdk/include${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"
export LIBRARY_PATH="$exam_sdk/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
catkin_make -j2 -l2 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DLIVOX_SDK_LIBRARY="$exam_sdk/lib/liblivox_sdk_static.a" \
  -DLIVOX_SDK_INCLUDE_DIR="$exam_sdk/include"
source devel/setup.bash
rospack find exam_bringup
rospack find fast_lio
rospack find exam_fastlio
printf '\n编译完成。运行 bash start_fastlio_navigation.sh 启动手动导航。\n'
