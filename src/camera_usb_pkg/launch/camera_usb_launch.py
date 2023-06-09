# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import argparse
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='usb_cam demo')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    args, unknown = parser.parse_known_args(sys.argv[4:])
    
    # parser1 = argparse.ArgumentParser(description='usb_cam demo')
    # parser1.add_argument('-n', '--node-name', dest='node_name', type=str,
    #                     help='name for device', default='usb_cam_1')

    # args1, unknown = parser1.parse_known_args(sys.argv[4:])

    # parser2 = argparse.ArgumentParser(description='usb_cam demo')
    # parser2.add_argument('-n', '--node-name', dest='node_name', type=str,
    #                     help='name for device', default='usb_cam_1')

    # args2, unknown = parser2.parse_known_args(sys.argv[4:])

    # parser3 = argparse.ArgumentParser(description='usb_cam demo')
    # parser3.add_argument('-n', '--node-name', dest='node_name', type=str,
    #                     help='name for device', default='usb_cam_1')

    # args3, unknown = parser3.parse_known_args(sys.argv[4:])

    # parser4 = argparse.ArgumentParser(description='usb_cam demo')
    # parser4.add_argument('-n', '--node-name', dest='node_name', type=str,
    #                     help='name for device', default='usb_cam_1')

    # args4, unknown = parser4.parse_known_args(sys.argv[4:])
    # get path to params fileusb_cam_dir = get_package_share_directory('usb_cam')

    # get path to params file

    usb_cam_dir = get_package_share_directory('camera_usb_pkg')

    params_path = os.path.join(
        usb_cam_dir,
        'config',
        'params.yaml'
    )
    # params_path_1 = '/home/cityu2022/camera_ws/config/params_1.yaml'
    # params_path_2 = '/home/cityu2022/camera_ws/config/params_2.yaml'
    # params_path_3 = '/home/cityu2022/camera_ws/config/params_3.yaml'
    # params_path_4 = '/home/cityu2022/camera_ws/config/params_4.yaml'

    node_name = args.node_name
    # node_name1 = args1.node_name
    # node_name2 = args2.node_name
    # node_name3 = args3.node_name
    # node_name4 = args4.node_name

    print(params_path)
    ld.add_action(Node(
        package='usb_cam', executable='usb_cam_node_exe', output='screen',
        name=node_name,
        namespace='/sensing/camera/traffic_light',
        parameters=[params_path]
        ))
    # ld.add_action(Node(
    #     package='usb_cam', executable='usb_cam_node_exe', output='screen',
    #     name=node_name1,
    #     namespace='usb_cam_1',
    #     parameters=[params_path_1]
    #     ))
    # ld.add_action(Node(
    #     package='usb_cam', executable='usb_cam_node_exe', output='screen',
    #     name=node_name2,
    #     namespace='usb_cam_2',
    #     parameters=[params_path_2]
    #     ))
    # ld.add_action(Node(
    #     package='usb_cam', executable='usb_cam_node_exe', output='screen',
    #     name=node_name3,
    #     namespace='usb_cam_3',
    #     parameters=[params_path_3]
    #     ))
    # ld.add_action(Node(
    #     package='usb_cam', executable='usb_cam_node_exe', output='screen',
    #     name=node_name4,
    #     namespace='usb_cam_4',
    #     parameters=[params_path_4]
    #     ))
    # ld.add_action(Node(
    #    package='usb_cam', executable='show_image.py', output='screen',
        # namespace=ns,
        # arguments=[image_manip_dir + "/data/mosaic.jpg"])
        # remappings=[('image_in', 'image_raw')]
    #    ))

    return ld
