"""Sensor fusion launch file: LiDAR clustering + EKF multi-target tracking."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('sil_sensor_fusion'), 'config', 'sensor_fusion.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the fusion node.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS 2 log level.'),

        Node(
            package='sil_sensor_fusion',
            executable='sil_sensor_fusion_node',
            name='sil_sensor_fusion',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
