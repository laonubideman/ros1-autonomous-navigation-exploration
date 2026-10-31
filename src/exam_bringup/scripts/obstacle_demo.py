#!/usr/bin/env python3
# 通过 Gazebo 服务加入或移除实体箱体；用于演示地图中尚未出现的障碍。
"""Insert, move, or remove a real box absent from the reference map."""
import argparse
import math
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, SetModelState, SpawnModel
from geometry_msgs.msg import Pose


# 生成带实体碰撞几何的箱体 SDF，不能仅用 RViz Marker 代替障碍。
def box_sdf(width=0.8, length=0.8, height=0.8, static=True):
    return '''<sdf version="1.6"><model name="box"><static>{3}</static><link name="body">
      <gravity>false</gravity><kinematic>{4}</kinematic>
      <collision name="collision"><geometry><box><size>{0} {1} {2}</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>{0} {1} {2}</size></box></geometry>
      <material><ambient>0.9 0.15 0.15 1</ambient><diffuse>0.9 0.15 0.15 1</diffuse></material>
      </visual></link></model></sdf>'''.format(
          width, length, height, str(static).lower(), str(not static).lower())


def delete_if_present(name):
    """Best-effort cleanup so repeated demonstrations remain one command."""
    rospy.wait_for_service('/gazebo/delete_model', timeout=15)
    return rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)(name)


def spawn(name, x, y, moving=False):
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=15)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, 0.4
    pose.orientation.w = 1.0
    return rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(
        name, box_sdf(static=not moving), '', pose, 'world')


def move(name, center_x, center_y, axis, distance, speed, duration):
    """Move a collision box sinusoidally; lidar and DWA must react online."""
    rospy.wait_for_service('/gazebo/set_model_state', timeout=15)
    setter = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    amplitude = max(0.05, distance)
    omega = max(0.01, speed) / amplitude
    start = rospy.get_time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and (duration <= 0 or rospy.get_time()-start < duration):
        elapsed = rospy.get_time()-start
        offset = amplitude*math.sin(omega*elapsed)
        velocity = amplitude*omega*math.cos(omega*elapsed)
        state = ModelState(model_name=name, reference_frame='world')
        state.pose.position.x, state.pose.position.y, state.pose.position.z = center_x, center_y, 0.4
        state.pose.orientation.w = 1.0
        if axis == 'x':
            state.pose.position.x += offset
            state.twist.linear.x = velocity
        else:
            state.pose.position.y += offset
            state.twist.linear.y = velocity
        result = setter(state)
        if not result.success:
            raise RuntimeError(result.status_message)
        rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['add', 'move', 'remove'])
    parser.add_argument('--x', type=float, default=2.0)
    parser.add_argument('--y', type=float, default=4.0)
    parser.add_argument('--axis', choices=['x', 'y'], default='x')
    parser.add_argument('--range', dest='distance', type=float, default=1.4,
                        help='movement amplitude in metres')
    parser.add_argument('--speed', type=float, default=0.35,
                        help='maximum movement speed in metres/second')
    parser.add_argument('--duration', type=float, default=45.0,
                        help='seconds to move; zero means until Ctrl+C')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('temporary_obstacle_demo', anonymous=True)
    name = 'exam_temporary_box'
    if args.action == 'remove':
        result = delete_if_present(name)
    elif args.action == 'move':
        # 先清理同名模型，使验收命令可重复运行。
        delete_if_present(name)
        result = spawn(name, args.x, args.y, moving=True)
        if not result.success:
            print(result.status_message)
            raise SystemExit(1)
        print('Moving collision obstacle started; Ctrl+C stops its motion.')
        move(name, args.x, args.y, args.axis, args.distance, args.speed, args.duration)
        result = delete_if_present(name)
    else:
        result = spawn(name, args.x, args.y)
    print(result.status_message)
    if not result.success:
        raise SystemExit(1)
