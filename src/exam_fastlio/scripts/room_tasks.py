#!/usr/bin/env python3
# 四房间任务进度；只使用实时地图与当前定位，不读取场景真值。
import math
import numpy as np
from scipy import ndimage


class RoomTasks:
    def __init__(self, rooms, minimum=0.95):
        self.rooms = [dict(room, visited=False, coverage=0.0, complete=False,
                           stable=0) for room in rooms]
        self.minimum = minimum
        self.active = None
        self.last_stamp = None

    # 要实际进入房间内部；隔着门观测到房间不能算完成访问。
    def update(self, grid, resolution, origin, position, stamp):
        xx = origin[0] + (np.arange(grid.shape[1])+0.5)*resolution
        yy = origin[1] + (np.arange(grid.shape[0])+0.5)*resolution
        # 从雷达测得的封闭占用轮廓填充箱体内部，不将不可进入的实体内部算作未知空地。
        blocked = ndimage.binary_fill_holes(ndimage.binary_closing(grid >= 65))
        clear = ndimage.distance_transform_edt(~blocked)*resolution > 0.35 if np.any(blocked) else np.ones(grid.shape, dtype=bool)
        for room in self.rooms:
            x0, y0, x1, y1 = room['bounds']
            # 边界内留出 0.35 米车体间距；未观测空地仍保留在覆盖率分母中。
            mask = (yy[:, None] >= y0+0.35)&(yy[:, None] < y1-0.35)&(xx >= x0+0.35)&(xx < x1-0.35)
            expected = round((x1-x0-0.7)/resolution)*round((y1-y0-0.7)/resolution)
            eligible = max(1, expected-np.count_nonzero(mask&~clear))
            room['coverage'] = float(min(1.0, np.count_nonzero((grid >= 0)&mask&clear)/eligible))
            if x0+0.6 <= position[0] <= x1-0.6 and y0+0.6 <= position[1] <= y1-0.6:
                room['visited'] = True
            if stamp != self.last_stamp:
                ready = room['visited'] and room['coverage'] >= self.minimum
                room['stable'] = room['stable']+1 if ready else 0
                room['complete'] = room['stable'] >= 3
        self.last_stamp = stamp
        remaining = [room for room in self.rooms if not room['complete']]
        if self.active not in [room['id'] for room in remaining]:
            self.active = min(remaining, key=lambda room: math.hypot(
                (room['bounds'][0]+room['bounds'][2])/2-position[0],
                (room['bounds'][1]+room['bounds'][3])/2-position[1]))['id'] if remaining else None

    def report(self):
        return [dict(id=r['id'], visited=r['visited'], coverage=round(r['coverage'], 4),
                     complete=r['complete']) for r in self.rooms]

    # 优先完成当前房间；未见到入口时继续沿已知安全前沿靠近该房间。
    def candidates(self, frontiers, grid, resolution, origin, position):
        if self.active is None:
            return []
        room = next(r for r in self.rooms if r['id'] == self.active)
        x0, y0, x1, y1 = room['bounds']
        inside = [g for g in frontiers if x0 <= g['x'] < x1 and y0 <= g['y'] < y1]
        goals = [dict(g, room=self.active) for g in inside]
        safe = (grid == 0)&(ndimage.distance_transform_edt(grid < 65)*resolution >= 0.5)
        cells = np.argwhere(safe)
        xy = origin+(cells[:, ::-1]+0.5)*resolution
        valid = (xy[:, 0] >= x0+0.6)&(xy[:, 0] <= x1-0.6)&(xy[:, 1] >= y0+0.6)&(xy[:, 1] <= y1-0.6)
        points = xy[valid]
        # 完全建好图仍需实际访问：目标从已知安全空地选取，而非直接发未知的房间中心。
        # 优先选从当前位姿可尽快进入的多个候选点；深处中心点会让中继规划
        # 在房门外反复换前沿。候选点全部由实时地图产生，不使用预设门坐标。
        if not room['visited']:
            if len(points):
                center = np.array([(x0+x1)/2, (y0+y1)/2])
                order = np.argsort(np.linalg.norm(points-position, axis=1)+
                                   0.05*np.linalg.norm(points-center, axis=1))
                visits = []
                for point in points[order]:
                    if all(np.linalg.norm(point-np.array([old['x'], old['y']])) >= 0.8
                           for old in visits):
                        visits.append(dict(x=float(point[0]), y=float(point[1]),
                            yaw=math.atan2(point[1]-position[1], point[0]-position[0]),
                            score=float(np.linalg.norm(point-position)), frontier_m=0.0,
                            task='room_visit', room=self.active))
                    if len(visits) == 8:
                        break
                goals = visits+goals
        # 箱体后的未知小块可能没有满足旧阈值的前沿；从已知空地选择多个不同观察位置。
        # 位置随实时未知区变化，不使用预设房间巡航点，也不穿过未知空地。
        if room['visited'] and room['coverage'] < self.minimum:
            xx = origin[0]+(np.arange(grid.shape[1])+0.5)*resolution
            yy = origin[1]+(np.arange(grid.shape[0])+0.5)*resolution
            region = (yy[:,None] >= y0+0.35)&(yy[:,None] < y1-0.35)&(xx >= x0+0.35)&(xx < x1-0.35)
            blocked = ndimage.binary_fill_holes(ndimage.binary_closing(grid >= 65))
            clear = ndimage.distance_transform_edt(~blocked)*resolution > 0.35 if np.any(blocked) else np.ones(grid.shape, dtype=bool)
            labels, count = ndimage.label(region&clear&(grid < 0))
            observations = []
            points = points[np.linalg.norm(points-position, axis=1) > 0.6]
            for group in range(1, count+1):
                unknown = np.argwhere(labels == group)
                if len(unknown)*resolution**2 < 0.10:
                    continue
                center = origin+(unknown.mean(axis=0)[::-1]+0.5)*resolution
                order = np.argsort(np.linalg.norm(points-center, axis=1)+0.1*np.linalg.norm(points-position, axis=1))
                chosen = []
                for point in points[order]:
                    if all(np.linalg.norm(point-old) >= 1.0 for old in chosen):
                        chosen.append(point)
                        observations.append(dict(x=float(point[0]), y=float(point[1]),
                            yaw=math.atan2(center[1]-point[1], center[0]-point[0]),
                            score=float(np.linalg.norm(point-center)), frontier_m=0.0,
                            task='room_observe', room=self.active))
                    if len(chosen) == 4:
                        break
            goals.extend(sorted(observations, key=lambda g: g['score']))
        outside = [g for g in frontiers if g not in inside]
        # 已完成房间的残余前沿不再作为任务目标，避免在房门处反复折返。
        completed = [r['bounds'] for r in self.rooms if r['complete']]
        outside = [g for g in outside if not any(a <= g['x'] < c and b <= g['y'] < d
                                                for a,b,c,d in completed)]
        outside.sort(key=lambda g: g['score']+1.5*math.hypot(
            max(x0-g['x'], 0, g['x']-x1), max(y0-g['y'], 0, g['y']-y1)))
        return goals+outside
