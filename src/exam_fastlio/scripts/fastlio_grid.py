#!/usr/bin/env python3
# 将 FAST-LIO2 配准点云投影为实时二维占用地图；自由空间来自射线观测，不读取场景真值。
"""Ray-trace FAST-LIO2 registered clouds into an online 20 m occupancy grid."""
import threading
import math
import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


class Grid:
    # 初始化对象状态，建立本模块需要的发布、订阅、缓存或验收记录。
    def __init__(self):
        self.latest = None
        self.lock = threading.Lock()
        self.start = [rospy.get_param('~start_x', 2.0), rospy.get_param('~start_y', 2.0)]
        yaw = rospy.get_param('~start_yaw', 0.0)
        self.rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        # 栅格为 400 乘 400，分辨率 0.05 米，覆盖固定 20 米乘 20 米场景。
        self.odds = np.zeros(400*400, dtype=np.int16)
        self.observed = np.zeros(400*400, dtype=bool)
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        self.publisher = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.message = OccupancyGrid()
        self.message.header.frame_id = 'map'
        self.message.info.resolution = 0.05
        self.message.info.width = self.message.info.height = 400
        self.message.info.origin.orientation.w = 1.0
        self.publish()
        rospy.Subscriber('/fastlio/cloud_registered', PointCloud2, self.receive, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update)

    # 只缓存最新点云，耗时栅格更新交给定时器，避免堵塞订阅回调。
    def receive(self, message):
        with self.lock:
            self.latest = message

    # 将点的平面坐标转为栅格索引，过滤地图外的点并去重。
    def indices(self, points):
        cells = np.floor(points[:,:2]/0.05).astype(np.int32)
        good = (cells[:,0]>=0)&(cells[:,0]<400)&(cells[:,1]>=0)&(cells[:,1]<400)
        return np.unique(cells[good,1]*400+cells[good,0])

    # 处理一帧配准点云，用下向射线清除自由空间、按高度标记障碍并更新地图。
    def update(self, event):
        with self.lock:
            message, self.latest = self.latest, None
        if message is None:
            return
        try:
            sensor = self.tf.lookup_transform('map','exam_lidar',message.header.stamp,rospy.Duration(0.2)).transform.translation
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            return
        # 按 PointCloud2 字段偏移解析三维坐标，保留 FAST-LIO2 的实际点云内容。
        offsets = {f.name:f.offset for f in message.fields}
        dtype = np.dtype({'names':['x','y','z'], 'formats':['<f4']*3,
                          'offsets':[offsets[n] for n in ('x','y','z')], 'itemsize':message.point_step})
        raw = np.frombuffer(message.data,dtype=dtype,count=message.width*message.height)
        # 把 camera_init 点云旋转和平移到 map，与 launch 的初始静态 TF 一致。
        xy = np.column_stack((raw['x'], raw['y'])) @ self.rotation.T + self.start
        points = np.column_stack((xy, raw['z']+0.078))
        points = points[np.all(np.isfinite(points),axis=1)]
        origin = np.array([sensor.x,sensor.y])
        # Downward beams clear space without looking over the arena's low boxes.
        # 仅使用低于传感器的回波清除射线路径，减少越过箱体后误清障碍。
        downward = points[(points[:,2]<sensor.z-0.005)&(points[:,2]>-0.2)]
        if len(downward):
            delta = downward[:,:2]-origin
            distances = np.linalg.norm(delta,axis=1)
            angles = (np.arctan2(delta[:,1],delta[:,0])+np.pi)/(2*np.pi)
            bins = np.clip((angles*360).astype(int),0,359)
            order = np.argsort(distances)
            farthest = np.full(360,-1,dtype=int)
            farthest[bins[order]] = order
            ends = downward[farthest[farthest>=0],:2]
            delta = ends-origin
            steps = np.maximum(1,np.ceil(np.linalg.norm(delta,axis=1)/0.025).astype(int))
            k = np.arange(steps.max())[:,None]
            fractions = k/steps[None,:]
            rays = origin+fractions[:,:,None]*delta[None,:,:]
            free = self.indices(rays[k<steps[None,:]])
            self.observed[free] = True
            self.odds[free] = np.maximum(-4,self.odds[free]-1)
        # 在平地场景中按点高度区分障碍；坡道和低矮物体需要额外可通行性处理。
        occupied = self.indices(points[(points[:,2]>0.10)&(points[:,2]<2.4)])
        self.observed[occupied] = True
        self.odds[occupied] = np.minimum(6,self.odds[occupied]+3)
        self.publish()

    # 将观测状态编码成未知负一、空地零、占用一百的 OccupancyGrid 并发布。
    def publish(self):
        # 未观测和证据不足的格子仍保持未知；不能预先涂成自由空间。
        grid = np.full(len(self.odds),-1,dtype=np.int8)
        grid[self.observed&(self.odds<=0)] = 0
        grid[self.observed&(self.odds>=2)] = 100
        self.message.header.stamp = rospy.Time.now()
        self.message.data = grid.tolist()
        self.publisher.publish(self.message)


if __name__ == '__main__':
    rospy.init_node('fastlio_grid')
    Grid()
    rospy.spin()
