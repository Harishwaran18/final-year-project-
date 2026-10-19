"""CAN gateway launch file.

dry_run:=true (the default) encodes frames but does not transmit, so the stack
can be launched on a host with no vcan0 interface. Create one with:

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('sil_can_gateway'), 'config', 'can_gateway.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'interface_name', default_value='vcan0',
            description='SocketCAN interface to transmit on.'),
        DeclareLaunchArgument(
            'dry_run', default_value='true',
            description='Encode frames without transmitting them.'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Parameter file for the CAN gateway.'),
        DeclareLaunchArgument(
            'log_level', default_value='info',
            description='ROS 2 log level.'),

        Node(
            package='sil_can_gateway',
            executable='sil_can_gateway_node',
            name='sil_can_gateway',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'interface_name': LaunchConfiguration('interface_name'),
                    'dry_run': LaunchConfiguration('dry_run'),
                },
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
    ])
