# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ubuntu/IEEE_ws/devel_isolated/map_example;/home/ubuntu/IEEE_ws/devel_isolated/mining_map;/home/ubuntu/IEEE_ws/devel_isolated/mecanum_odometry;/home/ubuntu/IEEE_ws/devel_isolated/mecanum_ik;/home/ubuntu/IEEE_ws/devel_isolated/mecanum_description;/home/ubuntu/IEEE_ws/devel_isolated/keyboard_control;/home/ubuntu/IEEE_ws/devel_isolated/keyboard;/home/ubuntu/IEEE_ws/devel_isolated/imu_tf;/home/ubuntu/IEEE_ws/devel_isolated/ieee_master;/home/ubuntu/IEEE_ws/devel_isolated/cv_camera;/home/ubuntu/IEEE_ws/devel_isolated/apriltag_tf;/home/ubuntu/IEEE_ws/devel_isolated/april_tag_detection;/home/ubuntu/WitStandardProtocol_JY901/ROS/wit/wit_ros_ws/devel;/home/ubuntu/IEEE_ws/devel_isolated/apriltag_ros;/home/ubuntu/IEEE_ws/devel;/home/ubuntu/catkin_ws/devel;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ubuntu/IEEE_ws/devel_isolated/motor_controller/env.sh')

output_filename = '/home/ubuntu/IEEE_ws/build_isolated/motor_controller/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)