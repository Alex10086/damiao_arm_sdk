import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("damiao_arm_bringup")
    xacro_path = os.path.join(pkg_share, "urdf", "damiao_arm.urdf.xacro")
    controllers_yaml = os.path.join(pkg_share, "config", "damiao_controllers.yaml")

    mappings = {
        "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
        "can_interface": LaunchConfiguration("can_interface").perform(context),
        "enable_fd": LaunchConfiguration("enable_fd").perform(context),
        "has_gripper": LaunchConfiguration("has_gripper").perform(context),
        "state_timeout_ms": LaunchConfiguration("state_timeout_ms").perform(context),
        "command_timeout_ms": LaunchConfiguration("command_timeout_ms").perform(context),
        "motor_timeout_reg": LaunchConfiguration("motor_timeout_reg").perform(context),
        "use_usb_bridge": LaunchConfiguration("use_usb_bridge").perform(context),
        "serial_port": LaunchConfiguration("serial_port").perform(context),
        "baudrate": LaunchConfiguration("baudrate").perform(context),
        "default_kp": LaunchConfiguration("default_kp").perform(context),
        "default_kd": LaunchConfiguration("default_kd").perform(context),
        "gripper_kp": LaunchConfiguration("gripper_kp").perform(context),
        "gripper_kd": LaunchConfiguration("gripper_kd").perform(context),
    }

    robot_description = xacro.process_file(xacro_path, mappings=mappings).toprettyxml(indent="  ")
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_yaml],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller = LaunchConfiguration("robot_controller").perform(context)
    arm_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    has_gripper = LaunchConfiguration("has_gripper")
    gripper_ctrl_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(has_gripper),
        output="screen",
    )

    delay_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[arm_ctrl_spawner, gripper_ctrl_spawner],
        )
    )

    return [robot_state_pub, control_node, jsb_spawner, delay_arm_after_jsb]


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        DeclareLaunchArgument("enable_fd", default_value="false"),
        DeclareLaunchArgument("has_gripper", default_value="true"),
        DeclareLaunchArgument("state_timeout_ms", default_value="500"),
        DeclareLaunchArgument("command_timeout_ms", default_value="100"),
        DeclareLaunchArgument("motor_timeout_reg", default_value="0"),
        DeclareLaunchArgument("use_usb_bridge", default_value="false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="921600"),
        DeclareLaunchArgument("default_kp", default_value="25.0"),
        DeclareLaunchArgument("default_kd", default_value="1.0"),
        DeclareLaunchArgument("gripper_kp", default_value="8.0"),
        DeclareLaunchArgument("gripper_kd", default_value="0.2"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=[
                "joint_trajectory_controller",
                "forward_position_controller",
                "forward_velocity_controller",
                "forward_effort_controller",
            ],
        ),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=_launch_setup)])
