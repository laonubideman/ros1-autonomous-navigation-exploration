#!/usr/bin/env python3
# 检查运动命令和传感器有效性，在延迟与制动期间检查车身扫掠范围，必要时输出零速度。
"""Limit commands and, in navigation mode, check fresh sensors and stopping sweeps."""
import math
import time
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class VelocityGuard:
    # 初始化对象状态，建立本模块需要的发布、订阅、缓存或验收记录。
    def __init__(self):
        self.command = Twist()
        self.received_at = 0.0
        self.max_linear = rospy.get_param('~max_linear', 0.25)
        self.require_scan = rospy.get_param('~require_scan', False)
        self.observation = None
        self.odometry = None
        self.last_reason = None
        self.publisher = rospy.Publisher('/cmd_vel_sim', Twist, queue_size=1)
        self.status = rospy.Publisher('/exam/guard_status', String, queue_size=1, latch=True)
        rospy.Subscriber('/cmd_vel', Twist, self.receive, queue_size=1)
        if self.require_scan:
            footprint = rospy.get_param('~footprint')
            padding = rospy.get_param('~footprint_padding', 0.025) + 0.02
            self.bounds = (min(p[0] for p in footprint) - padding,
                           max(p[0] for p in footprint) + padding,
                           min(p[1] for p in footprint) - padding,
                           max(p[1] for p in footprint) + padding)
            self.tf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tf)
            rospy.Subscriber('/scan', LaserScan, self.receive_scan, queue_size=1)
            rospy.Subscriber('/odom', Odometry, self.receive_odom, queue_size=1)
        rospy.on_shutdown(lambda: self.publisher.publish(Twist()))

    # 接收并限制底盘运动命令，同时记录命令接收时间。
    def receive(self, message):
        command = Twist()
        if math.isfinite(message.linear.x) and math.isfinite(message.angular.z):
            command.linear.x = max(-self.max_linear, min(self.max_linear, message.linear.x))
            command.angular.z = max(-1.0, min(1.0, message.angular.z))
        self.command = command
        self.received_at = time.monotonic()

    # 缓存实际里程计速度和接收时间，供失效停车与制动检查使用。
    def receive_odom(self, message):
        v, w = message.twist.twist.linear.x, message.twist.twist.angular.z
        if math.isfinite(v) and math.isfinite(w):
            self.odometry = (v, w, time.monotonic(), message.header.stamp.to_sec())

    # 把激光点变换到车体坐标，过滤无效距离并记录数据时间。
    def receive_scan(self, message):
        try:
            transform = self.tf.lookup_transform('base_footprint', message.header.frame_id,
                                                  rospy.Time(0), rospy.Duration(0.1)).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.observation = None
            return
        q = transform.rotation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        points = []
        valid = False
        for index, distance in enumerate(message.ranges):
            if distance == math.inf:
                valid = True
            elif math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                valid = True
                angle = message.angle_min + index * message.angle_increment + yaw
                points.append((transform.translation.x + distance * math.cos(angle),
                               transform.translation.y + distance * math.sin(angle)))
        self.observation = (points, time.monotonic(), message.header.stamp.to_sec()) if valid else None

    # 先考虑控制和传感器延迟，再模拟减速停车，逐步检查保守车身矩形与激光点是否重叠。
    def sweep_clear(self, points, v, w):
        # Conservatively retain velocity for sensing/control delay, then brake.
        delay, brake_linear, brake_angular, step = 0.35, 0.4, 1.0, 0.025
        horizon = delay + max(abs(v) / brake_linear, abs(w) / brake_angular)
        x = y = yaw = 0.0
        x0, x1, y0, y1 = self.bounds
        reach = abs(v) * horizon + max(math.hypot(a, b) for a in (x0, x1) for b in (y0, y1))
        nearby = [(px, py) for px, py in points if px*px + py*py <= reach*reach]
        for index in range(int(math.ceil(horizon / step)) + 1):
            c, s = math.cos(yaw), math.sin(yaw)
            for px, py in nearby:
                dx, dy = px - x, py - y
                bx, by = c*dx + s*dy, -s*dx + c*dy
                if x0 <= bx <= x1 and y0 <= by <= y1:
                    return False
            braking_time = max(0.0, index * step - delay)
            speed = math.copysign(max(0.0, abs(v) - brake_linear * braking_time), v)
            turn = math.copysign(max(0.0, abs(w) - brake_angular * braking_time), w)
            midpoint_yaw = yaw + turn * step / 2
            x += speed * math.cos(midpoint_yaw) * step
            y += speed * math.sin(midpoint_yaw) * step
            yaw += turn * step
        return True

    # 依次检查命令超时、激光与 TF 过期、里程计过期以及制动扫掠碰撞风险。
    def reason(self):
        wall_now, ros_now = time.monotonic(), rospy.Time.now().to_sec()
        if wall_now - self.received_at >= 0.5:
            return 'command_timeout'
        if not self.require_scan:
            return 'ok'
        scan, odom = self.observation, self.odometry
        if scan is None or wall_now - scan[1] > 0.6 or not -0.1 <= ros_now - scan[2] <= 0.6:
            return 'scan_or_tf_stale'
        if odom is None or wall_now - odom[2] > 0.6 or not -0.1 <= ros_now - odom[3] <= 0.6:
            return 'odometry_stale'
        candidates = [(self.command.linear.x, self.command.angular.z), odom[:2]]
        if not all(self.sweep_clear(scan[0], v, w) for v, w in candidates):
            return 'obstacle_in_stopping_sweep'
        return 'ok'

    # 持续检查保护条件，发布允许的运动速度或零速度。
    def run(self):
        while not rospy.is_shutdown():
            reason = self.reason()
            self.publisher.publish(self.command if reason == 'ok' else Twist())
            if reason != self.last_reason:
                self.status.publish(String(data=reason))
                self.last_reason = reason
            time.sleep(0.05)


if __name__ == '__main__':
    rospy.init_node('velocity_guard')
    VelocityGuard().run()
