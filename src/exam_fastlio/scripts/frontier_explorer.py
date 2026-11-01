#!/usr/bin/env python3
# 独立的自主探索上层；从当前 TF 位姿寻找可达前沿，向 move_base 发送目标，复用原有避障。
"""Select reachable map frontiers and let the existing move_base drive there."""
import json
import math
import time
from collections import deque
import threading
import sys
from pathlib import Path as FilePath

import actionlib
import numpy as np
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid,Path
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.srv import GetPlan
from scipy import ndimage
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
# catkin 的 devel 入口是转发脚本，显式加载实际源码旁的任务模块。
sys.path.insert(0, str(FilePath(__file__).resolve().parent))
from room_tasks import RoomTasks


# 按当前位姿选取已知安全连通区域，筛除小块未知噪声，返回已知一侧的前沿目标。
def find_frontiers(grid, resolution, origin, position, clearance=0.50):
    """Return free-side goals in the component containing the current robot."""
    # 只对达到面积阈值的未知区域探索，减少细小噪声造成的循环。
    unknown = grid < 0
    unknown_labels, _ = ndimage.label(unknown)
    sizes = np.bincount(unknown_labels.ravel())
    unknown &= sizes[unknown_labels]*resolution**2 >= 0.6
    # 欧氏距离变换给出到已知障碍的距离；目标选在保留 0.50 米参考点间距的空地。
    safe = (grid == 0)&(ndimage.distance_transform_edt(grid < 65)*resolution >= clearance)
    labels, _ = ndimage.label(safe)
    cells = np.argwhere(safe)
    if not len(cells):
        return [], False
    robot = (np.array(position)-origin)/resolution-0.5
    squared = (cells[:,1]-robot[0])**2+(cells[:,0]-robot[1])**2
    seed = cells[np.argmin(squared)]
    if squared.min()*resolution**2 > 0.6**2:
        return [], False
    # 只搜索当前位置可达的已知安全连通区域，避免跨墙选点。
    reachable = labels == labels[tuple(seed)]
    frontier = reachable&ndimage.binary_dilation(unknown)
    groups, count = ndimage.label(frontier,structure=np.ones((3,3)))
    distance = ndimage.distance_transform_edt(~unknown)*resolution
    # 目标向已知空地一侧退让，使底盘能到达并继续观测未知区域。
    retreat = np.argwhere(reachable&(distance >= 0.30)&(distance <= 0.80))
    goals = []
    if not len(retreat):
        return goals, True
    for group in range(1,count+1):
        boundary = np.argwhere(groups == group)
        if len(boundary)*resolution < 0.6:
            continue
        middle = boundary[np.argmin(np.sum((boundary-boundary.mean(axis=0))**2,axis=1))]
        away = np.sum((retreat-middle)**2,axis=1)
        cell = retreat[np.argmin(away)]
        if away.min()*resolution**2 > 0.9**2:
            continue
        xy = origin+(cell[::-1]+0.5)*resolution
        target = origin+(middle[::-1]+0.5)*resolution
        length = len(boundary)*resolution
        goals.append({'x':float(xy[0]),'y':float(xy[1]),
                      'yaw':math.atan2(target[1]-xy[1],target[0]-xy[0]),
                      'frontier_m':length,
                      'score':math.hypot(*(xy-position))-0.3*min(length,10.0)})
    return sorted(goals,key=lambda g:g['score']), True


# 在保留转向间距的已知空地内进行四邻域广度优先搜索，恢复供中间目标使用的路径。
def safe_route(grid, resolution, origin, position, target):
    """Find relay points around narrow shortcuts without changing move_base."""
    # 欧氏距离变换给出到已知障碍的距离；目标选在保留 0.50 米参考点间距的空地。
    safe = (grid == 0)&(ndimage.distance_transform_edt(grid < 65)*resolution >= 0.50)
    cells = np.argwhere(safe)
    if not len(cells):
        return np.empty((0,2))
    robot = (np.asarray(position)-origin)/resolution-0.5
    seed = cells[np.argmin(np.sum((cells[:,::-1]-robot)**2,axis=1))]
    end = np.floor((np.asarray(target)-origin)/resolution).astype(int)[::-1]
    height,width = grid.shape
    parents = np.full(grid.shape,-1,dtype=np.int32)
    parents[tuple(seed)] = seed[0]*width+seed[1]
    queue = deque([tuple(seed)])
    while queue and parents[tuple(end)] < 0:
        row,col = queue.popleft()
        for r,c in ((row-1,col),(row+1,col),(row,col-1),(row,col+1)):
            if 0<=r<height and 0<=c<width and safe[r,c] and parents[r,c]<0:
                parents[r,c] = row*width+col
                queue.append((r,c))
    if parents[tuple(end)] < 0:
        return np.empty((0,2))
    route = [end]
    while not np.array_equal(route[-1],seed):
        route.append(np.array(divmod(int(parents[tuple(route[-1])]),width)))
    return origin+(np.asarray(route[::-1])[:,::-1]+0.5)*resolution


# 复查规划路径的边界、占用、未知区与障碍间距；起点附近允许短距离退出不安全位置。
def path_is_safe(grid, resolution, origin, position, path):
    if not path:
        return False
    xy = np.array([(p.pose.position.x,p.pose.position.y) for p in path])
    cells = np.floor((xy-origin)/resolution).astype(int)
    valid = (cells[:,0]>=0)&(cells[:,0]<grid.shape[1])&(cells[:,1]>=0)&(cells[:,1]<grid.shape[0])
    if not np.all(valid):
        return False
    rows,cols = cells[:,1],cells[:,0]
    unknown_labels,_ = ndimage.label(grid<0)
    sizes = np.bincount(unknown_labels.ravel())
    large_unknown = (grid<0)&(sizes[unknown_labels]*resolution**2>=0.6)
    distance = ndimage.distance_transform_edt(grid<65)*resolution
    # 起点附近的短路径允许退出较近障碍位置，后续路径仍必须满足间距。
    beyond_start = np.linalg.norm(xy-position,axis=1)>0.3
    return bool(np.all((grid[rows,cols]<65)&~large_unknown[rows,cols]&
                       ((distance[rows,cols]>=0.50)|~beyond_start)))


# 以地图分辨率采样机器人到目标的直线，防止局部控制直接切过墙角或箱体。
def line_is_safe(grid, resolution, origin, position, target):
    steps = int(math.ceil(np.linalg.norm(np.asarray(target)-position)/resolution))+1
    path = []
    for x,y in np.linspace(position,target,steps):
        point = PoseStamped()
        point.pose.position.x,point.pose.position.y = x,y
        path.append(point)
    return path_is_safe(grid,resolution,origin,position,path)


# 逐格倒序寻找前方约两米内最远的可见安全中间点，避免固定步长漏掉门口转折。
def relay_targets(grid, resolution, origin, position, route):
    travelled = np.r_[0,np.cumsum(np.linalg.norm(np.diff(route,axis=0),axis=1))]
    furthest = min(len(route)-1,int(np.searchsorted(travelled,2.0)))
    for index in range(furthest,0,-1):
        point = route[index]
        if np.linalg.norm(point-position)>0.20 and line_is_safe(grid,resolution,origin,position,point):
            yield point


class Explorer:
    # 初始化对象状态，建立本模块需要的发布、订阅、缓存或验收记录。
    def __init__(self):
        self.map = None
        self.map_received = 0.0
        self.manual = False
        self.goal = None
        self.blacklist = []
        self.sent = self.succeeded = self.failed = 0
        self.start_xy = None
        self.costmap = self.current_path = None
        self.cost_lock = threading.Lock()
        self.replans = 0
        self.frontier_target = None
        # exploration.launch 默认加载四房间任务；不加载 rooms 时沿用全图前沿探索。
        rooms = rospy.get_param('~rooms', [])
        self.rooms = RoomTasks(rooms, rospy.get_param('~room_min_coverage', 0.95)) if rooms else None
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)
        # 上层只发送 move_base action 目标，速度仍由原有 A 星、DWA 和保护节点生成。
        self.client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        self.plan = rospy.ServiceProxy('/move_base/make_plan',GetPlan)
        self.status = rospy.Publisher('/exploration/status',String,queue_size=1,latch=True)
        self.markers = rospy.Publisher('/exploration/frontiers',MarkerArray,queue_size=1,latch=True)
        self.room_markers = rospy.Publisher('/exploration/rooms',MarkerArray,queue_size=1,latch=True)
        rospy.Subscriber('/map',OccupancyGrid,self.receive_map,queue_size=1)
        rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,self.receive_costmap,queue_size=1)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates',OccupancyGridUpdate,self.receive_costmap_update,queue_size=1)
        rospy.Subscriber('/move_base/GlobalPlanner/plan',Path,self.receive_path,queue_size=1)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.manual_goal,queue_size=1)
        rospy.on_shutdown(self.stop)

    # 缓存最新实时地图，并记录墙钟接收时间供数据新鲜度检查。
    def receive_map(self, message):
        self.map, self.map_received = message,time.monotonic()

    # 标记 RViz 手动接管，由主循环取消本节点自己的探索目标并退出。
    def manual_goal(self, message):
        self.manual = True

    # 缓存全局代价地图；探索只融合致命障碍，避免重复使用膨胀费用。
    def receive_costmap(self, message):
        with self.cost_lock:
            self.costmap = np.asarray(message.data,dtype=np.int16).reshape(message.info.height,message.info.width)

    # 在锁保护下把增量代价地图写回缓存，供下一次决策使用。
    def receive_costmap_update(self, message):
        with self.cost_lock:
            if self.costmap is not None:
                self.costmap[message.y:message.y+message.height,message.x:message.x+message.width] = np.asarray(message.data).reshape(message.height,message.width)

    # 保存 move_base 当前全局路径，行驶期间持续检查更新后的路线。
    def receive_path(self, message):
        self.current_path = message.poses

    # 取消本节点自己的探索动作并清除候选标记，不影响 RViz 新目标。
    def stop(self):
        # cancel_goal targets this client's UUID; it never cancels an RViz goal.
        # 仅取消本 action 客户端的目标编号，不能取消刚由 RViz 创建的新目标。
        if self.goal is not None:
            self.client.cancel_goal()
            self.goal = None
        self.markers.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    # 发布可供终端和验收程序读取的 JSON 状态，不直接控制底盘速度。
    def publish_status(self, state, **details):
        if self.rooms is not None:
            details.update(active_room=self.rooms.active, rooms=self.rooms.report(),
                           completed_rooms=sum(r['complete'] for r in self.rooms.rooms))
            markers = []
            for index, room in enumerate(self.rooms.rooms):
                x0, y0, x1, y1 = room['bounds']
                border = Marker()
                border.header.frame_id = 'map'
                border.ns, border.id, border.type = 'room_tasks', index*2, Marker.LINE_STRIP
                border.pose.orientation.w = 1
                border.scale.x = 0.06
                border.color.r = 0.2 if room['complete'] else 1.0
                border.color.g, border.color.b, border.color.a = 0.9, 0.2, 0.9
                border.points = [Point(x=x, y=y, z=0.10) for x, y in
                                 ((x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0))]
                label = Marker()
                label.header, label.ns, label.id = border.header, border.ns, index*2+1
                label.type, label.pose.orientation.w = Marker.TEXT_VIEW_FACING, 1
                label.pose.position.x, label.pose.position.y, label.pose.position.z = (x0+x1)/2, y1-0.4, 0.4
                label.scale.z, label.color = 0.40, border.color
                label.text = '%s %d%% %s' % (room['id'], round(room['coverage']*100),
                                             'OK' if room['complete'] else ('VISITED' if room['visited'] else ''))
                markers.extend((border, label))
            self.room_markers.publish(MarkerArray(markers=markers))
        self.status.publish(String(data=json.dumps(dict(state=state,start_xy=self.start_xy,
            sent=self.sent,succeeded=self.succeeded,failed=self.failed,replans=self.replans,goal=self.goal,**details))))

    # 读取当前位置或当前 TF 位姿，并换算平面朝向。
    def pose(self, frame):
        try:
            transform = self.tf.lookup_transform(frame,'base_footprint',rospy.Time(0),rospy.Duration(0.1))
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            return None
        if not -0.1 <= (rospy.Time.now()-transform.header.stamp).to_sec() <= 0.6:
            return None
        p = transform.transform.translation
        return (p.x,p.y)

    # 发布候选前沿 Marker，便于在 RViz 观察上层选择依据。
    def show_frontiers(self, candidates, frame):
        marker = Marker()
        marker.header.frame_id,marker.header.stamp = frame,rospy.Time.now()
        marker.ns,marker.id,marker.type = 'frontier_goals',0,Marker.SPHERE_LIST
        marker.pose.orientation.w = 1
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.g,marker.color.b,marker.color.a = 1,1,0.8
        marker.points = [Point(x=g['x'],y=g['y'],z=0.12) for g in candidates]
        self.markers.publish(MarkerArray(markers=[marker]))

    # 把失败目标所在前沿暂时列入黑名单，取消自己的动作并准备换点。
    def fail_goal(self):
        x,y = self.goal.get('frontier_xy',[self.goal['x'],self.goal['y']])
        self.blacklist.append((x,y,time.monotonic()+90))
        self.failed += 1
        self.frontier_target = None
        self.stop()

    # 运行探索状态机，处理地图更新、目标结果、失败换点和手动接管。
    def run(self):
        while not rospy.is_shutdown() and not self.client.wait_for_server(rospy.Duration(1)):
            self.publish_status('waiting_navigation')
        if rospy.is_shutdown():
            return
        while not rospy.is_shutdown():
            # 收到 RViz 目标后退出探索循环，手动目标继续由 move_base 执行。
            if self.manual:
                self.stop()
                self.publish_status('manual_override')
                return
            message = self.map
            position = self.pose(message.header.frame_id) if message is not None else None
            if position is None or time.monotonic()-self.map_received > 3:
                self.stop()
                self.publish_status('waiting_map_or_pose')
                time.sleep(0.2)
                continue
            if self.start_xy is None:
                self.start_xy = list(position)
                rospy.loginfo('Exploration starts from current pose (%.2f, %.2f)',*position)
            now = time.monotonic()
            grid = np.asarray(message.data,dtype=np.int16).reshape(message.info.height,message.info.width)
            observed = int(np.count_nonzero(grid>=0))
            origin = np.array([message.info.origin.position.x,message.info.origin.position.y])
            if self.rooms is not None:
                previous_room = self.rooms.active
                self.rooms.update(grid, message.info.resolution, origin, position, message.header.stamp.to_nsec())
                if previous_room is not None and previous_room != self.rooms.active:
                    self.stop()
                    self.frontier_target = None
                    self.next_plan = 0
                if self.rooms.active is None:
                    self.stop()
                    self.publish_status('complete', robot_xy=position, observed_cells=observed)
                    time.sleep(0.2)
                    continue
            # 融合实时致命障碍；其余膨胀费用由下层代价地图处理。
            with self.cost_lock:
                ready = self.costmap is not None
                if ready:
                    grid[self.costmap==100] = 100
            if not ready:
                self.publish_status('waiting_costmap')
                time.sleep(0.2)
                continue
            if self.goal is not None:
                state = self.client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    if 'frontier_xy' not in self.goal and observed <= self.goal_observed+100:
                        self.blacklist.append((self.goal['x'],self.goal['y'],now+90))
                    if 'frontier_xy' not in self.goal:
                        self.frontier_target = None
                    self.succeeded += 1
                    self.goal = None
                    self.next_plan = now+2
                elif state in (GoalStatus.ABORTED,GoalStatus.REJECTED,GoalStatus.LOST):
                    self.fail_goal()
                elif state in (GoalStatus.PREEMPTED,GoalStatus.RECALLED):
                    self.goal = None
                    self.publish_status('external_control')
                    return
                else:
                    # 地图持续变化时复查已发布的全局路径，发现不安全部分后重新规划。
                    if now-self.goal_at>2 and now>=getattr(self,'path_check_at',0):
                        self.path_check_at = now+1
                        if self.current_path is not None and not path_is_safe(grid,message.info.resolution,origin,position,self.current_path):
                            self.stop()
                            self.replans += 1
                            self.next_plan = 0
                            continue
                    distance = math.hypot(position[0]-self.goal['x'],position[1]-self.goal['y'])
                    if distance < self.best_distance-0.10 or observed > self.best_observed+100:
                        self.best_distance,self.best_observed,self.progress_at = distance,observed,now
                    # 目标长期没有距离或地图进展时取消，防止在同一处无限等待。
                    if now-self.progress_at > 45 or now-self.goal_at > 180:
                        self.fail_goal()
                    else:
                        self.publish_status('exploring',robot_xy=position,observed_cells=observed)
                        time.sleep(0.2)
                        continue
            if now < getattr(self,'next_plan',0):
                time.sleep(0.2)
                continue
            self.next_plan = now+2
            candidates,located = find_frontiers(grid,message.info.resolution,
                origin,np.array(position))
            if self.rooms is not None:
                candidates = self.rooms.candidates(candidates, grid, message.info.resolution, origin, np.array(position))
            self.show_frontiers(candidates,message.header.frame_id)
            self.blacklist = [b for b in self.blacklist if b[2] > now]
            candidates = [g for g in candidates if all(math.hypot(g['x']-x,g['y']-y)>0.8
                                                       for x,y,_ in self.blacklist)]
            # 短中间目标完成后优先推进同一个前沿，减少房间之间反复换向。
            if self.frontier_target is not None and candidates:
                closest = min(candidates,key=lambda g:math.hypot(g['x']-self.frontier_target[0],g['y']-self.frontier_target[1]))
                same_task = self.rooms is None or closest.get('room') == self.rooms.active or not any(g.get('room') == self.rooms.active for g in candidates)
                if same_task and math.hypot(closest['x']-self.frontier_target[0],closest['y']-self.frontier_target[1])<1.0:
                    candidates.remove(closest)
                    # 进入房间前不得被新前沿抢走优先级；已在房内时才允许
                    # 信息量更大的观察点排在中继目标之前。
                    entering = closest.get('task') == 'room_visit'
                    offset = 1 if not entering and self.rooms is not None and any(
                        g.get('room') == self.rooms.active for g in candidates) else 0
                    candidates.insert(offset,closest)
            selected = None
            for candidate in candidates:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = message.header.frame_id
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x,goal.target_pose.pose.position.y = candidate['x'],candidate['y']
                goal.target_pose.pose.orientation.z = math.sin(candidate['yaw']/2)
                goal.target_pose.pose.orientation.w = math.cos(candidate['yaw']/2)
                start = PoseStamped()
                start.header = goal.target_pose.header
                start.pose.position.x,start.pose.position.y = position
                start.pose.orientation.w = 1
                try:
                    path = self.plan(start,goal.target_pose,0.0).plan.poses
                except rospy.ServiceException:
                    break
                if path_is_safe(grid,message.info.resolution,origin,position,path) and line_is_safe(
                    grid,message.info.resolution,origin,position,[candidate['x'],candidate['y']]):
                    selected = (candidate,goal)
                    break
                # 全局路径不适合直接执行时，改用保留间距的栅格路线生成中间目标。
                route = safe_route(grid,message.info.resolution,origin,position,[candidate['x'],candidate['y']])
                if not len(route):
                    continue
                for point in relay_targets(grid,message.info.resolution,origin,position,route):
                    relay = dict(candidate,x=float(point[0]),y=float(point[1]),
                                 frontier_xy=[candidate['x'],candidate['y']])
                    goal.target_pose.pose.position.x,goal.target_pose.pose.position.y = point
                    yaw = math.atan2(point[1]-position[1],point[0]-position[0])
                    relay['yaw'] = yaw
                    goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
                    try:
                        path = self.plan(start,goal.target_pose,0.0).plan.poses
                    except rospy.ServiceException:
                        break
                    if path_is_safe(grid,message.info.resolution,origin,position,path):
                        selected = (relay,goal)
                        break
                if selected is not None:
                    break
            if selected is None:
                # 无安全起点时等待；存在暂时不可达前沿时重试；无候选前沿时报告完成。
                state = 'waiting_reachable_pose' if not located else ('retrying_frontiers' if candidates or self.blacklist else 'complete')
                if self.rooms is not None and state == 'complete':
                    state = 'rooms_incomplete'
                self.publish_status(state,observed_cells=observed,frontier_count=len(candidates))
                continue
            if self.manual or rospy.is_shutdown():
                continue
            self.goal,goal = selected
            self.frontier_target = self.goal.get('frontier_xy',[self.goal['x'],self.goal['y']])
            self.best_distance = math.hypot(position[0]-self.goal['x'],position[1]-self.goal['y'])
            self.best_observed,self.progress_at = observed,now
            self.goal_at = now
            self.goal_observed = observed
            self.current_path = None
            self.client.send_goal(goal)
            self.sent += 1
            rospy.loginfo('Frontier goal %d: (%.2f, %.2f)',self.sent,self.goal['x'],self.goal['y'])
            self.publish_status('exploring',robot_xy=position,observed_cells=observed)


if __name__ == '__main__':
    rospy.init_node('frontier_explorer')
    explorer = Explorer()
    try:
        explorer.run()
    finally:
        explorer.stop()
