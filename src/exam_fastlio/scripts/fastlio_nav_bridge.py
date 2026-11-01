#!/usr/bin/env python3
# 将 FAST-LIO2 的 IMU 位姿换算成平面车体位姿、速度及导航 TF。
"""Publish planar navigation odometry solely from FAST-LIO2's IMU pose."""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Bridge:
    # 初始化对象状态，建立本模块需要的发布、订阅、缓存或验收记录。
    def __init__(self):
        self.previous = None
        self.velocity = [0.0, 0.0]
        self.start_x = rospy.get_param('~start_x', 2.0)
        self.start_y = rospy.get_param('~start_y', 2.0)
        self.start_yaw = rospy.get_param('~start_yaw', 0.0)
        self.map_min = rospy.get_param('~map_min', -0.5)
        self.map_max = rospy.get_param('~map_max', 20.5)
        self.max_linear_jump = rospy.get_param('~max_linear_jump', 1.0)
        self.max_angular_jump = rospy.get_param('~max_angular_jump', 3.0)
        self.publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.health = rospy.Publisher('/exam/fastlio_health', String, queue_size=1, latch=True)
        self.last_health = None
        self.tf = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/fastlio/odometry', Odometry, self.receive, queue_size=1)

    # 仅在状态变化时发布诊断，避免高频重复日志。
    def report(self, state):
        if state != self.last_health:
            self.health.publish(String(data=state))
            self.last_health = state

    # 验证 LIO 位姿后输出平面导航状态，过滤非有限数值和逆序时间。
    def receive(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in values):
            self.report('rejected_non_finite_pose')
            return
        # 扣除旋转后的 IMU 安装偏移，再按 launch 初始位姿对齐 map；不读取 sim/odom。
        local_x = p.x - 0.078 * 2 * (q.x*q.z + q.w*q.y)
        local_y = p.y - 0.078 * 2 * (q.y*q.z - q.w*q.x)
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        x = self.start_x + c*local_x - s*local_y
        y = self.start_y + s*local_x + c*local_y
        yaw = self.start_yaw + math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = message.header.stamp.to_sec()
        # 官方地图范围是 20x20 m。不可能的位置或帧间跳变说明 LIO
        # 初始重力估计失败；此时停止发布导航里程计，令安全层制动。
        if not (self.map_min <= x <= self.map_max and self.map_min <= y <= self.map_max):
            self.report('rejected_pose_outside_map')
            return
        if self.previous is None and math.hypot(x-self.start_x, y-self.start_y) > 0.5:
            self.report('rejected_invalid_initial_pose')
            return
        if self.previous is not None:
            old_stamp, old_x, old_y, old_yaw = self.previous
            dt = stamp-old_stamp
            if dt <= 0:
                self.report('rejected_non_monotonic_time')
                return
            distance = math.hypot(x-old_x, y-old_y)
            turn_delta = abs(math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw)))
            if distance > max(0.10, self.max_linear_jump*dt):
                self.report('rejected_linear_pose_jump')
                return
            if turn_delta > max(0.15, self.max_angular_jump*dt):
                self.report('rejected_angular_pose_jump')
                return
            # 用连续 LIO 位姿的时间差估计转向和车体前向速度。
            turn = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))/dt
            speed = ((x-old_x)*math.cos(yaw)+(y-old_y)*math.sin(yaw))/dt
            self.velocity = [0.4*speed+0.6*self.velocity[0], 0.4*turn+0.6*self.velocity[1]]
        self.previous = (stamp,x,y,yaw)
        self.report('healthy')
        # 发布供 DWA 读取的平面里程计，三维 LIO 结果仍保留在原话题。
        output = Odometry()
        output.header.stamp, output.header.frame_id = message.header.stamp, 'odom'
        output.child_frame_id = 'base_footprint'
        output.pose.pose.position.x, output.pose.pose.position.y = x,y
        output.pose.pose.orientation.z, output.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
        output.pose.covariance = message.pose.covariance
        output.twist.twist.linear.x, output.twist.twist.angular.z = self.velocity
        self.publisher.publish(output)
        # 唯一发布 odom 到 base_footprint 的导航 TF，避免与仿真插件竞争。
        transform = TransformStamped()
        transform.header, transform.child_frame_id = output.header, output.child_frame_id
        transform.transform.translation.x, transform.transform.translation.y = x,y
        transform.transform.rotation = output.pose.pose.orientation
        self.tf.sendTransform(transform)


if __name__ == '__main__':
    rospy.init_node('fastlio_nav_bridge')
    Bridge()
    rospy.spin()
