# Copyright 2026 SIL AEB Maintainers
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

# SIL AEB stack: CARLA bridge -> sensor fusion -> AEB controller -> CAN gateway.
#
#   ros2 launch sil_bringup sil_aeb_stack.launch.py
#   ros2 launch sil_bringup sil_aeb_stack.launch.py use_carla:=true
#   ros2 launch sil_bringup sil_aeb_stack.launch.py publish_scenario:=true
#
# The default configuration is fully synthetic: the CARLA bridge synthesises a
# LiDAR sweep and camera detections from an actor list, so the whole pipeline
# runs without a simulator. Set use_carla:=true to relay a real CARLA instance
# instead, and dry_run:=false once a vcan0 interface exists.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_share = get_package_share_directory('sil_carla_bridge')
    fusion_share = get_package_share_directory('sil_sensor_fusion')
    aeb_share = get_package_share_directory('sil_aeb_controller')
    can_share = get_package_share_directory('sil_can_gateway')

    use_carla = LaunchConfiguration('use_carla')
    dry_run = LaunchConfiguration('dry_run')
    can_interface = LaunchConfiguration('can_interface')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_carla', default_value='false',
            description='Relay a real CARLA instance instead of synthesising sensors.'),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Encode CAN frames without transmitting them.'),
        DeclareLaunchArgument(
            'can_interface', default_value='vcan0',
            description='SocketCAN interface for the CAN gateway.'),
        DeclareLaunchArgument(
            'publish_scenario', default_value='false',
            description='Publish a synthetic CARLA actor stream.'),
        DeclareLaunchArgument(
            'scenario', default_value='lead_vehicle_braking',
            description='Synthetic scenario name.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS 2 log level for every node.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bridge_share, 'launch', 'carla_bridge.launch.py')),
            launch_arguments={
                'synthesis_mode': 'false' if use_carla else 'true',
                'log_level': log_level,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fusion_share, 'launch', 'sensor_fusion.launch.py')),
            launch_arguments={'log_level': log_level}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aeb_share, 'launch', 'aeb_controller.launch.py')),
            launch_arguments={'log_level': log_level}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(can_share, 'launch', 'can_gateway.launch.py')),
            launch_arguments={
                'interface_name': can_interface,
                'dry_run': dry_run,
                'log_level': log_level,
            }.items(),
        ),

        # A synthetic actor feed, so the stack has something to react to when no
        # simulator is attached.
        Node(
            package='sil_carla_bridge',
            executable='carla_scenario_publisher',
            name='carla_scenario_publisher',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_scenario')),
            parameters=[{
                'scenario': LaunchConfiguration('scenario'),
                'rate_hz': 20.0,
            }],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
