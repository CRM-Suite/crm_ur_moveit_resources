import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    launch_servo = LaunchConfiguration("launch_servo")
    launch_notebook = LaunchConfiguration("launch_notebook")
    launch_moveitpy = LaunchConfiguration("launch_moveitpy")

    launch_servo_arg = DeclareLaunchArgument(
        "launch_servo", 
        default_value="false",
        description="Launch Servo?"
    )

    launch_notebook_arg = DeclareLaunchArgument(
        "launch_notebook", 
        default_value="false",
        description="Launch Jupyter Notebook?"
    )
    
    launch_moveitpy_arg = DeclareLaunchArgument(
        "launch_moveitpy",
        default_value="false",
        description="Launch MoveIt with Python Example?"
    )

    example_file = DeclareLaunchArgument(
        "example_file",
        default_value="example3.py",
        description="Python API tutorial file name",
    )

    moveit_config = (
        MoveItConfigsBuilder("crm_robot", package_name="crm_moveit_config")
            .moveit_cpp(file_path=get_package_share_directory("crm_moveit_config") + "/config/notebook.yaml")
            .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": ":0"},
    )

    moveit_py_node = Node(
        name="moveit_py",
        package="crm_moveit_config",
        condition=IfCondition(launch_moveitpy),
        executable=LaunchConfiguration("example_file"),
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    servo_yaml = load_yaml("crm_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        condition=IfCondition(launch_servo),
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    notebook_dir = os.path.join(get_package_share_directory("crm_moveit_config"), "examples")
    start_notebook = ExecuteProcess(
        cmd=["cd {} && python3 -m notebook --allow-root".format(notebook_dir)],
        shell=True,
        output="screen",
        condition=IfCondition(launch_notebook)
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("crm_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            launch_servo_arg,
            launch_notebook_arg,
            launch_moveitpy_arg,
            example_file,
            move_group_node,
            moveit_py_node,
            start_notebook,
            rviz_node,
            servo_node,
            joy_node,
        ]
    )