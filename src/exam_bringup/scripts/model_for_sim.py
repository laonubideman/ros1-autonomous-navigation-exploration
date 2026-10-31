#!/usr/bin/env python3
# 展开官方 TurtleBot3 Xacro，调整仿真插件，并在 SDF 中加入独立碰撞验收传感器。
"""Load the upstream robot unchanged on disk and adapt its runtime plugins."""
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
import rospkg
import xacro

package = rospkg.RosPack().get_path('turtlebot3_description')
path = os.path.join(package, 'urdf', 'turtlebot3_waffle_pi.urdf.xacro')
root = ET.fromstring(xacro.process_file(path).toxml())
for gazebo in list(root.findall('gazebo')):
    # The first version needs wheels, IMU and laser; skip camera rendering.
    if gazebo.get('reference') == 'camera_rgb_frame':
        root.remove(gazebo)
        continue
    for plugin in gazebo.findall('plugin'):
        if plugin.get('filename') == 'libgazebo_ros_diff_drive.so':
            plugin.find('commandTopic').text = '/cmd_vel_sim'
            plugin.find('wheelAcceleration').text = '0.5'
        if plugin.get('filename') == 'libgazebo_ros_imu.so':
            plugin.find('updateRate').text = '200'
# 固定关节合并后再加入接触传感器，确保碰撞名称与生成的 SDF 一致。
if '--sdf' in sys.argv:
    # Add sensors after fixed-link merging so their collision references are exact.
    with tempfile.TemporaryDirectory(prefix='exam_robot_') as temporary:
        urdf_path = os.path.join(temporary, 'robot.urdf')
        ET.ElementTree(root).write(urdf_path, encoding='unicode')
        converted = subprocess.run(['gz', 'sdf', '-p', urdf_path], check=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, timeout=15)
        text = converted.stdout.replace('ignition::corrected_offsets', 'ignition_corrected_offsets')
        root = ET.fromstring(text)
    for link in root.findall('model/link'):
        collisions = link.findall('collision')
        if not collisions:
            continue
        sensor = ET.SubElement(link, 'sensor', type='contact', name='contact_' + link.get('name'))
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '50'
        contact = ET.SubElement(sensor, 'contact')
        for collision in collisions:
            ET.SubElement(contact, 'collision').text = collision.get('name')
        plugin = ET.SubElement(sensor, 'plugin', name='bumper_' + link.get('name'), filename='libgazebo_ros_bumper.so')
        ET.SubElement(plugin, 'bumperTopicName').text = '/exam/contacts'
        ET.SubElement(plugin, 'frameName').text = 'world'
print(ET.tostring(root, encoding='unicode'))
