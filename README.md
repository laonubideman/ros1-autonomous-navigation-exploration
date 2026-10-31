# ROS1 四房间探索工作空间

包含 20×20 米场景、TurtleBot3 Waffle Pi 模型、FAST-LIO2、Livox ROS 驱动与 SDK 源码、实时二维建图、A 星与 DWA 导航、四房间自主探索，以及原版参考地图导航。

## 1. 环境与依赖

使用 **Ubuntu 20.04 x86_64 + ROS Noetic**。原生 Ubuntu、虚拟机和 WSL2 Ubuntu 20.04 均在 Ubuntu 终端执行以下命令。RViz 需要桌面显示环境，WSL2 可使用 WSLg。Gazebo 仅在后台仿真，不打开 Gazebo 窗口。

已经安装 ROS Noetic 并配置好软件源的机器，跳过下面的软件源配置。

首次配置 ROS 软件源，使用官方 [ros-apt-source 1.3.0 focal 包](https://github.com/ros-infrastructure/ros-apt-source/releases/tag/1.3.0)：

```bash
sudo apt update
sudo apt install -y curl ca-certificates software-properties-common unzip
sudo add-apt-repository -y universe
mkdir -p ~/Downloads
cd ~/Downloads
curl -fL -o ros-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros-apt-source_1.3.0.focal_all.deb
sudo dpkg -i ros-apt-source.deb
sudo apt update
```

安装编译和运行依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-move-base ros-noetic-dwa-local-planner \
  ros-noetic-global-planner ros-noetic-map-server \
  ros-noetic-velodyne-gazebo-plugins ros-noetic-teleop-twist-keyboard \
  ros-noetic-pcl-ros ros-noetic-eigen-conversions \
  build-essential cmake pkg-config unzip \
  python3-dev python3-rospkg python3-numpy python3-scipy \
  libpcl-dev libeigen3-dev libboost-all-dev libapr1-dev
```

## 2. 解压与编译

```bash
cd ~/ros1_official_ws
bash build.sh
source devel/setup.bash
```


`build.sh` 先编译包内 Livox SDK，再编译所有 catkin 功能包；SDK 安装到本工作空间的 `vendor/livox-sdk-install`。不需要另行克隆项目，也不需要复制其他机器的 `build`、`devel` 或 SDK 二进制。

后续命令均在**本次解压的工作空间根目录**执行。根目录启动脚本会自动加载环境；直接执行 `roslaunch`、`rosrun` 时，需要在当前终端先运行 `source devel/setup.bash`。

## 3. 启动 FAST-LIO2 实时建图与目标导航

```bash
cd ~/ros1_official_ws
bash start_fastlio_navigation.sh
```

等待 RViz 中出现小车、激光和二维地图，使用 **2D Nav Goal** 点击目标并拖动设置朝向，小车通过 A 星与 DWA 导航到目标。此入口默认不开启自主探索。

可以在全新启动时设置安全空地中的起点；朝向单位为弧度：

```bash
bash start_fastlio_navigation.sh start_x:=16.2 start_y:=10.1 start_yaw:=2.95
```

这些参数用于仿真出生点和 LIO 初始坐标对齐。导航运行后，正常驾驶到其他位置再开启探索即可，不需要重新设置起点。

直接 launch 的等价入口：

```bash
source devel/setup.bash
roslaunch exam_fastlio fastlio_navigation.launch
```

### 动态障碍避障验收

保持 FAST-LIO2 导航运行，先在 RViz 中把目标点设到小车前方较远的空地，再另开终端运行：

```bash
cd ~/ros1_official_ws
source devel/setup.bash
rosrun exam_bringup obstacle_demo.py move \
  --x 2.0 --y 4.0 --axis x --range 1.4 --speed 0.35 --duration 45
```

该命令在仿真中生成带碰撞体的红色移动箱，不是 RViz 图形。二维激光和局部代价地图实时观测它，DWA 负责减速、停车或绕行。命令结束后会自动删除箱体；如中途退出后需手动清理：

```bash
rosrun exam_bringup obstacle_demo.py remove
```

## 4. 独立启动自主探索

保持上一节 FAST-LIO2 导航运行，**另开终端**进入同一工作空间。

### 四房间探索（默认）

```bash
cd ~/ros1_official_ws
bash start_exploration.sh
```

探索从开启时的当前位置开始，不会重置小车。上层决策选择未完成房间；实际进入且实时地图估计的可通行区域覆盖率达到 95%，连续三次不同地图更新满足条件，才记为该房间完成。四间完成后停止探索运动。

房间边界与覆盖阈值位于 `src/exam_fastlio/config/rooms.yaml`。RViz 的 `Four room task progress` 显示任务进度；运行时可查看：

```bash
source devel/setup.bash
rostopic echo /exploration/status
```

直接 launch 的等价入口：

```bash
source devel/setup.bash
roslaunch exam_fastlio exploration.launch
```

### 原全图前沿探索

不启用四房间任务判据时，运行：

```bash
bash start_exploration.sh rooms_file:=
```

两种探索模式一次只启动一种。探索过程中，在 RViz 下发新的 **2D Nav Goal** 会结束探索并接管导航。

## 5. 启动原版程序

启动前先关闭正在运行的探索和基础导航，避免共用同一 ROS master。

### 原版参考地图导航

```bash
cd ~/ros1_official_ws
bash start_reference_navigation.sh
```

此模式使用固定参考二维地图与仿真里程计，保留 A 星、DWA 和 RViz 目标导航，作为学习对照。

直接 launch 的等价入口：

```bash
source devel/setup.bash
roslaunch exam_bringup navigation.launch
```

### 最初的小车运动例程

```bash
cd ~/ros1_official_ws
source devel/setup.bash
roslaunch exam_bringup initial.launch
```

该模式不启动 `move_base`。另开终端执行键盘驾驶：

```bash
cd ~/ros1_official_ws
source devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

按键操作见终端提示；速度命令发布到 `/cmd_vel`。

## 6. 停止与切换

使用根目录脚本启动的程序：

```bash
# 已开启探索时，先停止探索；基础导航继续运行
bash stop_exploration.sh

# 停止基础导航
bash stop.sh
```

未开启探索时跳过 `stop_exploration.sh`。也可在各自启动终端按 **Ctrl+C**；直接 `roslaunch` 启动的程序使用各自终端的 **Ctrl+C**。

停止当前程序后，按需要选择新的入口：

```bash
# FAST-LIO2 实时建图与目标导航
bash start_fastlio_navigation.sh

# 或原版参考地图导航
bash start_reference_navigation.sh
```

两套基础导航一次只启动一种，不需要额外启动 `roscore`。重启 FAST-LIO2 导航会重新建图。

## 7. 建议验收顺序

1. 运行 `bash build.sh`，确认三个主要功能包都能被 `rospack` 找到。
2. 运行 `bash start_fastlio_navigation.sh`，驾驶小车一段距离，确认 RViz 同时显示三维点云、实时二维地图和轨迹。
3. 用 **2D Nav Goal** 下发跨区域目标，确认 A 星生成全局路径、DWA 控制小车到达。
4. 执行上述动态障碍命令，录制小车实时避障。
5. 运行 `bash start_exploration.sh`，确认任务从当前位置开始，`/exploration/status` 中四间房依次达到 95% 且最终进入 `complete`。

导航运行后可在另一终端执行只读自检：

```bash
cd ~/ros1_official_ws
source devel/setup.bash
python3 verify_runtime.py
```

自检会验证必需节点、IMU、原始三维点云、FAST-LIO2 配准点云与里程计、二维激光、实时地图和 TF 链。开启探索后再运行一次，还会校验探索状态。

FAST-LIO2 主入口不加载 `organizer_truth.pgm`；该图只供第 5 节的原版参考导航使用。免试验收应展示 FAST-LIO2 实时建图与探索主入口。
