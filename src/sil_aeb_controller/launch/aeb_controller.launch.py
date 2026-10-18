"""AEB controller launch file: TTC computation and the braking FSM."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('sil_aeb_controller'), 'config', 'aeb_controller.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the AEB controller.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS 2 log level.'),

        Node(
            package='sil_aeb_controller',
            executable='sil_aeb_controller_node',
            name='sil_aeb_controller',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
