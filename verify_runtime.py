#!/usr/bin/env python3
"""Read-only acceptance check for the running FAST-LIO navigation stack."""
import json
import math
import sys

import rosnode
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from std_msgs.msg import String


def receive(topic, message_type, timeout=12.0):
    message = rospy.wait_for_message(topic, message_type, timeout=timeout)
    print('[PASS] %-30s %s' % (topic, message_type.__name__))
    return message


def main():
    rospy.init_node('exam_runtime_verifier', anonymous=True, disable_signals=True)
    nodes = set(rosnode.get_node_names())
    expected = {'/gazebo', '/laserMapping', '/fastlio_nav_bridge',
                '/fastlio_grid', '/move_base', '/velocity_guard'}
    missing = sorted(expected-nodes)
    if missing:
        raise RuntimeError('missing ROS nodes: ' + ', '.join(missing))
    print('[PASS] required ROS nodes')

    receive('/fastlio/imu', Imu)
    receive('/fastlio/points', PointCloud2)
    cloud = receive('/fastlio/cloud_registered', PointCloud2)
    receive('/fastlio/odometry', Odometry)
    odometry = receive('/odom', Odometry)
    health = receive('/exam/fastlio_health', String)
    receive('/scan', LaserScan)
    grid = receive('/map', OccupancyGrid)

    position = odometry.pose.pose.position
    if not all(math.isfinite(value) for value in (position.x, position.y)):
        raise RuntimeError('navigation odometry position is not finite')
    if not (-0.5 <= position.x <= 20.5 and -0.5 <= position.y <= 20.5):
        raise RuntimeError('navigation odometry position is outside the 20 x 20 m arena')
    if health.data != 'healthy':
        raise RuntimeError('FAST-LIO bridge health is ' + health.data)
    print('[PASS] FAST-LIO navigation health  healthy')

    if cloud.width*cloud.height <= 0:
        raise RuntimeError('registered point cloud is empty')
    if (grid.info.width, grid.info.height) != (400, 400):
        raise RuntimeError('map must be 400 x 400 cells')
    if abs(grid.info.resolution-0.05) > 1e-9:
        raise RuntimeError('map resolution must be 0.05 m')
    known = sum(value >= 0 for value in grid.data)
    occupied = sum(value >= 65 for value in grid.data)
    if known < 500 or occupied < 10:
        raise RuntimeError('online map has too little observed geometry; drive for a few seconds and retry')
    print('[PASS] online map                 known=%d occupied=%d' % (known, occupied))

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    transform = tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(5))
    age = (rospy.Time.now()-transform.header.stamp).to_sec()
    if age > 1.0:
        raise RuntimeError('map -> base_footprint transform is stale')
    print('[PASS] map -> base_footprint TF   age=%.3fs' % age)

    if '/frontier_explorer' in nodes:
        status = receive('/exploration/status', String, timeout=5)
        json.loads(status.data)
        print('[PASS] exploration status is valid JSON')
    else:
        print('[INFO] exploration is not running; start_exploration.sh is checked separately')
    print('\nRuntime acceptance check passed.')


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('\n[FAIL] %s' % error, file=sys.stderr)
        raise SystemExit(1)
