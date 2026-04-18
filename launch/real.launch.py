"""Launch rov_pid_controller for the real BlueROV2: publishes normalized Twist
on `cmd_vel`, which bluerov2_controller maps to per-DOF PWM via MAVROS."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rov_pid_controller')
    params = os.path.join(pkg, 'config', 'controller.yaml')

    return LaunchDescription([
        Node(
            package='rov_pid_controller',
            executable='rov_pid_controller_node',
            namespace='rov_pid_controller',
            name='rov_pid_controller',
            parameters=[params, {
                'publish_wrench': False,
                'publish_cmd_vel': True,
            }],
            remappings=[
                # Joystick teleop (configured in m/s, rad/s) feeds the controller.
                ('cmd_vel_in', '/cmd_vel_joy'),
                # bluerov2_controller subscribes to the root-level /cmd_vel.
                ('cmd_vel', '/cmd_vel'),
            ],
            output='screen',
        ),
    ])
