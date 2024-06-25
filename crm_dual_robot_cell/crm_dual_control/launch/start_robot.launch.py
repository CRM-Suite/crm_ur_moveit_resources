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

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    headless_mode = LaunchConfiguration("headless_mode")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_arguments = []
    # Left UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_robot_ip",
            default_value="192.168.0.101",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_controller_file",
            default_value="left_ur3_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_tf_prefix",
            default_value="left_ur3_",
            description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_script_command_port",
            default_value="50002",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_trajectory_port",
            default_value="50003",
            description="Port that will be opened to forward trajectory commands from the driver to the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_reverse_port",
            default_value="50001",
            description="Port that will be opened to forward reverse commands from the driver to the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur3_script_sender_port",
            default_value="50005",
            description="Port that will be opened to forward script sender commands from the driver to the robot",
        )
    )

    # Right UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_robot_ip",
            default_value="192.168.0.100",
            description="IP address by which the robot can be reached.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_controller_file",
            default_value="right_ur3_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_tf_prefix",
            default_value="right_ur3_",
            description="tf_prefix of the joint names, useful for \
            multi-robot setup. If changed, also joint names in the controllers' configuration \
            have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_script_command_port",
            default_value="50010",
            description="Port that will be opened to forward script commands from the driver to the robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_trajectory_port",
            default_value="50009",
            description="Port that will be opened to forward trajectory commands from the driver to the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_reverse_port",
            default_value="50006",
            description="Port that will be opened to forward reverse commands from the driver to the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur3_script_sender_port",
            default_value="50007",
            description="Port that will be opened to forward script sender commands from the driver to the robot",
        )
    )

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. "
            "Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )

    ur_launch_dir = get_package_share_directory("crm_dual_control")

    left_ur3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_launch_dir, "launch", "ur_control.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": left_ur3_robot_ip,
            "runtime_config_package": "crm_dual_control",
            "controllers_file": left_ur3_controller_file,
            "description_package": "crm_dual_description",
            "description_file": "crm_dual.urdf.xacro",
            "tf_prefix": left_ur3_tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "headless_mode": headless_mode,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": launch_rviz,
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
            "runtime_config_package": "crm_dual_control",
            "controllers_file": right_ur3_controller_file,
            "description_package": "crm_dual_description",
            "description_file": "crm_dual.urdf.xacro",
            "tf_prefix": right_ur3_tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "headless_mode": headless_mode,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "launch_rviz": launch_rviz,
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
        declared_arguments
        + [
            left_ur3_with_namespace,
            right_ur3_with_namespace,
        ]
    )
