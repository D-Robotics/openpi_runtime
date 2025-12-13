# Copyright (c) 2025，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from ament_index_python.packages import get_package_prefix


def generate_launch_description():

    # args that can be set from the command line or a default will be used
    cam_serial_no_arg = DeclareLaunchArgument(
        "cam_serial_no", default_value=TextSubstitution(text="_234322306420")
    )
    cam_left_serial_no_arg = DeclareLaunchArgument(
        "cam_left_serial_no", default_value=TextSubstitution(text="_241122306184")
    )
    color_profile_launch_arg = DeclareLaunchArgument(
        "color_profile", default_value=TextSubstitution(text="1280x720x30")
    )

    cam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py')),
        launch_arguments={
            'serial_no': LaunchConfiguration('cam_serial_no'),
            'camera_namespace': 'camera',
            'camera_name': 'camera',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile')
        }.items()
    )

    cam_left_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch/rs_launch.py')),
        launch_arguments={
            'serial_no': LaunchConfiguration('cam_left_serial_no'),
            'camera_namespace': 'camera_left',
            'camera_name': 'camera_left',
            'rgb_camera.color_profile': LaunchConfiguration('color_profile')
        }.items()
    )

    # jpeg图片编码&发布pkg
    codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'ros',
            'codec_in_format': 'rgb8',
            'codec_out_format': 'jpeg',
            'codec_sub_topic': '/camera/camera/color/image_raw',
            'codec_pub_topic': '/image_jpeg'
        }.items()
    )

    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_only_show_image': 'True',
            'websocket_image_type': 'mjpeg'
        }.items()
    )

    # 算法pkg
    openpi_runtime_node = Node(
        package='openpi_runtime',
        executable='openpi_runtime_node',
        output='screen',
        parameters=[
            {"user_prompt": "beat block hammer"},
            {"max_limit_num": 50},
            {"state_sub_topic_name": "/joint_states"},
            {"camera_topic_name": "/camera/camera/color/image_raw"},
            {"camera_left_topic_name": "/camera_left/camera_left/color/image_raw"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([
        cam_serial_no_arg,
        cam_left_serial_no_arg,
        color_profile_launch_arg,
        # 图片发布pkg
        cam_node,
        # 左臂图片发布pkg
        cam_left_node,
        # 图片编解码&发布pkg
        codec_node,
        # 启动example pkg
        openpi_runtime_node,
        # 启动web展示pkg
        web_node
    ])

