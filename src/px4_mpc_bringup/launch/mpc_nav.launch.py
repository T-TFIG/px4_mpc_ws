"""Stage 3: full stack -- SITL + agent + MPC node + trajectory generator.

Run PX4 SITL separately first (`make px4_sitl gz_x500` from /PX4-Autopilot,
in its own terminal), then run this.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sitl_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('px4_mpc_bringup'),
                'launch', 'sitl_agent.launch.py')
        )
    )

    params_file = os.path.join(
        get_package_share_directory('px4_mpc_controller'), 'config', 'mpc_params.yaml')

    mpc_node = Node(
        package='px4_mpc_controller',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        parameters=[params_file],
    )

    trajectory_generator_node = Node(
        package='px4_mpc_controller',
        executable='trajectory_generator',
        name='trajectory_generator',
        output='screen',
        parameters=[params_file],
    )

    vehicle_pose_publisher_node = Node(
        package='px4_mpc_controller',
        executable='vehicle_pose_publisher',
        name='vehicle_pose_publisher',
        output='screen',
    )

    return LaunchDescription([
        sitl_agent_launch, mpc_node, trajectory_generator_node, vehicle_pose_publisher_node,
    ])
