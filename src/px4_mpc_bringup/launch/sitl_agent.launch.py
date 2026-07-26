"""Stage 1: launch the Micro XRCE-DDS Agent (the PX4 <-> ROS2 bridge).

Run PX4 SITL separately first (`make px4_sitl gz_x500` from /PX4-Autopilot,
in its own terminal) -- this only manages the ROS2-side bridge process.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    micro_xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
    )
    return LaunchDescription([micro_xrce_agent])
