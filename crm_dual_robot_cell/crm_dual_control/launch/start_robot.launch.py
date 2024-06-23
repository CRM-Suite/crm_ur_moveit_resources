import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    ur_type = LaunchConfiguration("ur_type")
    left_ur3_robot_ip = LaunchConfiguration("left_ur3_robot_ip")
    left_ur3_controller_file = LaunchConfiguration("left_ur3_controller_file")
    left_ur3_tf_prefix = LaunchConfiguration("left_ur3_tf_prefix")
    left_ur3_script_command_port = LaunchConfiguration("left_ur3_script_command_port")
    left_ur3_trajectory_port = LaunchConfiguration("left_ur3_trajectory_port")
    left_ur3_reverse_port = LaunchConfiguration("left_ur3_reverse_port")
    left_ur3_script_sender_port = LaunchConfiguration("left_ur3_script_sender_port")

    right_ur3_robot_ip = LaunchConfiguration("right_ur3_robot_ip")
    right_ur3_controller_file = LaunchConfiguration("right_ur3_controller_file")
    right_ur3_tf_prefix = LaunchConfiguration("right_ur3_tf_prefix")
    right_ur3_script_command_port = LaunchConfiguration("right_ur3_script_command_port")
    right_ur3_trajectory_port = LaunchConfiguration("right_ur3_trajectory_port")
    right_ur3_reverse_port = LaunchConfiguration("right_ur3_reverse_port")
    right_ur3_script_sender_port = LaunchConfiguration("right_ur3_script_sender_port")

    # # UR specific arguments
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur5",
        description="Type/series of used UR robot.",
        choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
    )
    left_ur3_robot_ip_arg = DeclareLaunchArgument(
        "left_ur3_robot_ip",
        default_value="172.17.0.2",
        description="IP address by which the robot can be reached.",
    )
    left_ur3_controller_file_arg = DeclareLaunchArgument(
        "left_ur3_controller_file",
        default_value="left_ur3_ur_controllers.yaml",
        description="YAML file with the controllers configuration.",
    )
    left_ur3_tf_prefix_arg = DeclareLaunchArgument(
        "left_ur3_tf_prefix",
        default_value="left_ur3_",
        description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
    )

    left_ur3_script_command_port_arg = DeclareLaunchArgument(
        "left_ur3_script_command_port",
        default_value="50002",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )
    left_ur3_trajectory_port_arg = DeclareLaunchArgument(
        "left_ur3_trajectory_port",
        default_value="50003",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )
    left_ur3_reverse_port_arg = DeclareLaunchArgument(
        "left_ur3_reverse_port",
        default_value="50001",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )
    left_ur3_script_sender_port_arg = DeclareLaunchArgument(
        "left_ur3_script_sender_port",
        default_value="50005",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )

    right_ur3_robot_ip_arg = DeclareLaunchArgument(
        "right_ur3_robot_ip",
        default_value="172.17.0.2",
        description="IP address by which the robot can be reached.",
    )
    right_ur3_controller_file_arg = DeclareLaunchArgument(
        "right_ur3_controller_file",
        default_value="right_ur3_ur_controllers.yaml",
        description="YAML file with the controllers configuration.",
    )
    right_ur3_tf_prefix_arg = DeclareLaunchArgument(
        "right_ur3_tf_prefix",
        default_value="right_ur3_",
        description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
    )
    right_ur3_script_command_port_arg = DeclareLaunchArgument(
        "right_ur3_script_command_port",
        default_value="50010",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )

    right_ur3_trajectory_port_arg = DeclareLaunchArgument(
        "right_ur3_trajectory_port",
        default_value="50009",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )
    right_ur3_reverse_port_arg = DeclareLaunchArgument(
        "right_ur3_reverse_port",
        default_value="50006",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )
    right_ur3_script_sender_port_arg = DeclareLaunchArgument(
        "right_ur3_script_sender_port",
        default_value="50007",
        description="Port that will be opened to forward script commands from the driver to the robot",
    )

    ur_launch_dir = get_package_share_directory("ur_robot_driver")

    left_ur3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "launch", "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": left_ur3_robot_ip,
            "controllers_file": left_ur3_controller_file,
            "tf_prefix": left_ur3_tf_prefix,
            "script_command_port": left_ur3_script_command_port,
            "trajectory_port": left_ur3_trajectory_port,
            "reverse_port": left_ur3_reverse_port,
            "script_sender_port": left_ur3_script_sender_port,
        }.items(),
    )

    left_ur3_with_namespace = GroupAction(
        actions=[PushRosNamespace("left_ur3"), left_ur3]
    )

    right_ur3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "launch", "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": right_ur3_robot_ip,
            "controllers_file": right_ur3_controller_file,
            "tf_prefix": right_ur3_tf_prefix,
            "script_command_port": right_ur3_script_command_port,
            "trajectory_port": right_ur3_trajectory_port,
            "reverse_port": right_ur3_reverse_port,
            "script_sender_port": right_ur3_script_sender_port,
        }.items(),
    )

    right_ur3_with_namespace = GroupAction(
        actions=[PushRosNamespace("right_ur3"), right_ur3]
    )

    return LaunchDescription(
        [
            ur_type_arg,
            left_ur3_robot_ip_arg,
            left_ur3_controller_file_arg,
            left_ur3_tf_prefix_arg,
            left_ur3_script_command_port_arg,
            left_ur3_trajectory_port_arg,
            left_ur3_reverse_port_arg,
            left_ur3_script_sender_port_arg,
            right_ur3_robot_ip_arg,
            right_ur3_controller_file_arg,
            right_ur3_tf_prefix_arg,
            right_ur3_script_command_port_arg,
            right_ur3_trajectory_port_arg,
            right_ur3_reverse_port_arg,
            right_ur3_script_sender_port_arg,
            # left_ur3_with_namespace,
            right_ur3_with_namespace,
        ]
    )
