"""Display the CAD arm using ros2_control Fake Hardware only."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("damiao_arm_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg_share, "rviz", "damiao_arm.rviz"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "damiao_arm.launch.py")
            ),
            launch_arguments={
                # Constants intentionally override any inherited launch configuration.
                "use_fake_hardware": "true",
                "use_usb_bridge": "false",
                "has_gripper": "false",
                "robot_controller": "joint_trajectory_controller",
                "start_rviz": LaunchConfiguration("start_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
            }.items(),
        ),
    ])
