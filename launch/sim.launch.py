"""Launch rov_pid_controller for OceanSim: publishes Wrench."""

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
                'publish_wrench': True,
                'publish_cmd_vel': False,
            }],
            remappings=[
                # Point at whichever odometry source you prefer at launch time.
                # Override the `odometry_topic` param for a different source.
                ('cmd_vel_in', '/cmd_vel_joy'),
                # OceanSim's ROS2ControlReceiver subscribes to this topic in
                # "force control" mode — see isaacsim.oceansim.utils.ros2_control.
                ('wrench', '/oceansim/robot/force_cmd'),
            ],
            output='screen',
        ),
    ])
