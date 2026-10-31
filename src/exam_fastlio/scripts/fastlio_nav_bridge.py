#!/usr/bin/env python3
# 将 FAST-LIO2 的 IMU 位姿换算成平面车体位姿、速度及导航 TF。
"""Publish planar navigation odometry solely from FAST-LIO2's IMU pose."""
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class Bridge:
    # 初始化对象状态，建立本模块需要的发布、订阅、缓存或验收记录。
    def __init__(self):
        self.previous = None
        self.velocity = [0.0, 0.0]
        self.start_x = rospy.get_param('~start_x', 2.0)
        self.start_y = rospy.get_param('~start_y', 2.0)
        self.start_yaw = rospy.get_param('~start_yaw', 0.0)
        self.publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('/fastlio/odometry', Odometry, self.receive, queue_size=1)

    # 验证 LIO 位姿后输出平面导航状态，过滤非有限数值和逆序时间。
    def receive(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in values):
            return
        # 扣除旋转后的 IMU 安装偏移，再按 launch 初始位姿对齐 map；不读取 sim/odom。
        local_x = p.x - 0.078 * 2 * (q.x*q.z + q.w*q.y)
        local_y = p.y - 0.078 * 2 * (q.y*q.z - q.w*q.x)
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        x = self.start_x + c*local_x - s*local_y
        y = self.start_y + s*local_x + c*local_y
        yaw = self.start_yaw + math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = message.header.stamp.to_sec()
        if self.previous is not None:
            old_stamp, old_x, old_y, old_yaw = self.previous
            dt = stamp-old_stamp
            if dt <= 0:
                return
            # 用连续 LIO 位姿的时间差估计转向和车体前向速度。
            turn = math.atan2(math.sin(yaw-old_yaw), math.cos(yaw-old_yaw))/dt
            speed = ((x-old_x)*math.cos(yaw)+(y-old_y)*math.sin(yaw))/dt
            self.velocity = [0.4*speed+0.6*self.velocity[0], 0.4*turn+0.6*self.velocity[1]]
        self.previous = (stamp,x,y,yaw)
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
