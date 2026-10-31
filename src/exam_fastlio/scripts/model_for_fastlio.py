#!/usr/bin/env python3
# 在官方小车运行时模型上增加三维雷达和带重力的 IMU，并隔离仿真真值里程计。
"""Extend the baseline model at runtime; leave its files and navigation untouched."""
import subprocess
import sys
import xml.etree.ElementTree as ET
import rospkg

baseline = rospkg.RosPack().get_path('exam_bringup') + '/scripts/model_for_sim.py'
sdf_mode = '--sdf' in sys.argv
root = ET.fromstring(subprocess.check_output(
    [sys.executable, baseline] + (['--sdf'] if sdf_mode else []), text=True))

# URDF 分支用于 RViz 模型与传感器固定 TF。
if not sdf_mode:
    link = ET.SubElement(root, 'link', name='exam_lidar')
    visual = ET.SubElement(link, 'visual')
    ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder', radius='0.05', length='0.04')
    ET.SubElement(ET.SubElement(visual, 'material', name='lio_lidar'), 'color', rgba='0.2 0.7 0.9 1')
    joint = ET.SubElement(root, 'joint', name='exam_lidar_mount', type='fixed')
    ET.SubElement(joint, 'parent', link='base_footprint')
    ET.SubElement(joint, 'child', link='exam_lidar')
    ET.SubElement(joint, 'origin', xyz='0 0 0.55', rpy='0 0 0')
# SDF 分支用于后台物理仿真及传感器插件。
else:
    model = root.find('model')
    for plugin in list(model.findall('plugin')):
        if plugin.get('filename') == 'libgazebo_ros_imu.so':
            model.remove(plugin)
        elif plugin.get('filename') == 'libgazebo_ros_diff_drive.so':
            # 仿真真值仅发布到 sim/odom，关闭其导航 TF，防止真值混入 FAST-LIO2 定位。
            plugin.find('odometryTopic').text = '/sim/odom'
            plugin.find('odometryFrame').text = 'world'
            plugin.find('publishOdomTF').text = 'false'
    for link in model.findall('link'):
        for sensor in list(link.findall("sensor[@type='imu']")):
            link.remove(sensor)
    # Gazebo versions differ in which side of a fixed joint keeps the merged
    # link name.  Prefer base_footprint, but remain usable when it is base_link.
    body = model.find("link[@name='base_footprint']")
    body_name = 'base_footprint'
    if body is None:
        body = model.find("link[@name='base_link']")
        body_name = 'base_link'
    if body is None:
        raise RuntimeError('Converted TurtleBot3 model has no base link')
    z_offset = 0.0 if body_name == 'base_footprint' else -0.010
    # 使用 IMU sensor 插件提供含重力加速度，满足 LIO 初始化的数据要求。
    imu = ET.SubElement(body, 'sensor', name='fastlio_imu', type='imu')
    ET.SubElement(imu, 'pose').text = '0 0 %.3f 0 0 0' % (0.078+z_offset)
    ET.SubElement(imu, 'always_on').text = 'true'
    ET.SubElement(imu, 'update_rate').text = '200'
    plugin = ET.SubElement(imu, 'plugin', name='fastlio_imu_ros', filename='libgazebo_ros_imu_sensor.so')
    for key, value in {'robotNamespace':'/', 'topicName':'/fastlio/imu', 'frameName':'imu_link',
                       'updateRateHZ':'200', 'gaussianNoise':'0.001',
                       'initialOrientationAsReference':'false'}.items():
        ET.SubElement(plugin, key).text = value
    # 雷达安装高 0.55 米，十赫兹，十六线；插件发布 FAST-LIO2 输入点云。
    lidar = ET.SubElement(body, 'sensor', name='fastlio_lidar', type='ray')
    ET.SubElement(lidar, 'pose').text = '0 0 %.3f 0 0 0' % (0.55+z_offset)
    ET.SubElement(lidar, 'always_on').text = 'true'
    ET.SubElement(lidar, 'visualize').text = 'false'
    ET.SubElement(lidar, 'update_rate').text = '10'
    ray = ET.SubElement(lidar, 'ray')
    scan = ET.SubElement(ray, 'scan')
    for name, samples, low, high in [('horizontal',360,-3.141592654,3.141592654),
                                     ('vertical',16,-0.261799388,0.261799388)]:
        axis = ET.SubElement(scan, name)
        for key, value in {'samples':samples, 'resolution':1, 'min_angle':low, 'max_angle':high}.items():
            ET.SubElement(axis, key).text = str(value)
    limits = ET.SubElement(ray, 'range')
    for key, value in {'min':'0.2', 'max':'16', 'resolution':'0.001'}.items():
        ET.SubElement(limits, key).text = value
    plugin = ET.SubElement(lidar, 'plugin', name='fastlio_lidar_ros', filename='libgazebo_ros_velodyne_laser.so')
    for key, value in {'robotNamespace':'/', 'topicName':'/fastlio/points', 'frameName':'exam_lidar',
                       'organize_cloud':'false', 'min_range':'0.2', 'max_range':'15',
                       'gaussianNoise':'0.005'}.items():
        ET.SubElement(plugin, key).text = value
print(ET.tostring(root, encoding='unicode'))
