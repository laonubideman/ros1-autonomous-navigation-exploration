#!/usr/bin/env python3
# 仅供 RViz 显示场景、障碍和轨迹；场景几何与模型真值不作为建图、定位或导航输入。
"""Show the fixed scene geometry and the simulated odometry trail in RViz."""
import json
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray


class SceneVisuals:
    # 初始化对象状态，建立本模块需要的发布、订阅、缓存或验收记录。
    def __init__(self):
        with open(rospy.get_param('~geometry_file'), encoding='utf-8') as stream:
            scene = json.load(stream)
        self.scene_pub = rospy.Publisher('/exam/scene', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('/exam/trajectory', Path, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/exam/obstacles', MarkerArray, queue_size=1, latch=True)
        self.obstacle_names = ('exam_temporary_box', 'exam_test_barrier')
        self.obstacle_signature = None
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.last_stamp = 0.0
        markers = MarkerArray()
        for index, (name, x0, y0, x1, y1, height) in enumerate(scene['rectangles']):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id = 'arena', index
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            marker.pose.position.x = (x0 + x1) / 2
            marker.pose.position.y = (y0 + y1) / 2
            marker.pose.position.z = height / 2
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = x1 - x0, y1 - y0, height
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.36, 0.48, 0.59, 0.85)
            if name.startswith('obstacle') or name == 'hall_post':
                marker.color.r, marker.color.g, marker.color.b = (0.86, 0.60, 0.26)
            markers.markers.append(marker)
        self.scene_pub.publish(markers)
        rospy.Subscriber('/odom', Odometry, self.receive, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.receive_models, queue_size=1)

    # 把临时物理箱体位置转换为 RViz 标记；该真值只用于显示。
    def receive_models(self, message):
        poses = {name: message.pose[message.name.index(name)]
                 for name in self.obstacle_names if name in message.name}
        signature = tuple((name, round(p.position.x, 4), round(p.position.y, 4),
                           round(p.orientation.z, 4)) for name, p in poses.items())
        if signature == self.obstacle_signature:
            return
        self.obstacle_signature = signature
        markers = MarkerArray()
        for index, name in enumerate(self.obstacle_names):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.ns, marker.id = 'temporary_obstacles', index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD if name in poses else Marker.DELETE
            if name in poses:
                marker.pose = poses[name]
                marker.scale.x, marker.scale.y, marker.scale.z = ((0.8, 0.8, 0.8) if index == 0 else (0.3, 4.0, 0.8))
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.95, 0.15, 0.15, 0.9)
            markers.markers.append(marker)
        self.obstacle_pub.publish(markers)

    # 从导航里程计采样显示轨迹，限频并限制轨迹长度。
    def receive(self, odom):
        stamp = odom.header.stamp.to_sec()
        if stamp - self.last_stamp < 0.1:
            return
        self.last_stamp = stamp
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = odom.header.stamp, 'map'
        pose.pose.position.x = odom.pose.pose.position.x
        pose.pose.position.y = odom.pose.pose.position.y
        # Raise the display line slightly above the reference map plane.
        pose.pose.position.z = odom.pose.pose.position.z + 0.02
        pose.pose.orientation = odom.pose.pose.orientation
        self.path.poses.append(pose)
        self.path.poses = self.path.poses[-3000:]
        self.path.header.stamp = odom.header.stamp
        self.path_pub.publish(self.path)


if __name__ == '__main__':
    rospy.init_node('scene_visuals')
    SceneVisuals()
    rospy.spin()
