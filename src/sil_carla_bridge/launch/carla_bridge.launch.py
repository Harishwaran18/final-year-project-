"""CARLA bridge launch file.

Synthesis mode (default) turns an actor list into a LiDAR sweep and camera
detections. Relay mode forwards the simulator's own sensor topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('sil_carla_bridge'), 'config', 'carla_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'synthesis_mode', default_value='true',
            description='Synthesise sensors from the actor list instead of relaying.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the CARLA bridge.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS 2 log level.'),

        Node(
            package='sil_carla_bridge',
            executable='sil_carla_bridge_node',
            name='sil_carla_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'synthesis_mode': LaunchConfiguration('synthesis_mode')},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
