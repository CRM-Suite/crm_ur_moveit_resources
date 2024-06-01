import os
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    # Declare launch arguments
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3",
        description="Type/series of used UR robot.",
        choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20"],
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
        description="Indicate whether robot is running with fake hardware mirroring command to its states.",
    )
    safety_limits_arg = DeclareLaunchArgument(
        "safety_limits",
        default_value="true",
        description="Enables the safety limits controller if true.",
    )
    safety_pos_margin_arg = DeclareLaunchArgument(
        "safety_pos_margin",
        default_value="0.15",
        description="The margin to lower and upper limits in the safety controller.",
    )
    safety_k_position_arg = DeclareLaunchArgument(
        "safety_k_position",
        default_value="20",
        description="k-position factor in the safety controller.",
    )
    description_package_arg = DeclareLaunchArgument(
        "description_package",
        default_value="ur_description",
        description="Description package with robot URDF/XACRO files.",
    )
    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value="ur.urdf.xacro",
        description="URDF/XACRO description file with the robot.",
    )
    moveit_config_package_arg = DeclareLaunchArgument(
        "moveit_config_package",
        default_value="ur_moveit_config",
        description="MoveIt config package with robot SRDF/XACRO files.",
    )
    moveit_config_file_arg = DeclareLaunchArgument(
        "moveit_config_file",
        default_value="ur.srdf.xacro",
        description="MoveIt SRDF/XACRO description file with the robot.",
    )
    warehouse_sqlite_path_arg = DeclareLaunchArgument(
        "warehouse_sqlite_path",
        default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
        description="Path where the warehouse database should be stored",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Make MoveIt to use simulation time.",
    )
    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value='',
        description="Prefix of the joint names, useful for multi-robot setup.",
    )
    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    launch_servo_arg = DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")

    # Initialize LaunchConfigurations
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
            .robot_description(file_path=get_package_share_directory("ur_description") + "/urdf/ur.urdf.xacro",
                mappings={
                    "robot_ip": "yyy.yyy.yyy.yyy",
                    "joint_limit_params": joint_limit_params,
                    "kinetic_params": kinematics_params,
                    "physical_params": physical_params,
                    "visual_params": visual_params,
                    "safety_limits": safety_limits,
                    "safety_pos_margin": safety_pos_margin,
                    "safety_k_position": safety_k_position,
                    "name": "ur",
                    "ur_type": ur_type,
                    "script_filename": "ros_control.urscript",
                    "input_recipe_filename": "rtde_input_recipe.txt",
                    "output_recipe_filename": "rtde_output_recipe.txt",
                    "prefix": prefix
                }
            )
            .robot_description_semantic(file_path=get_package_share_directory("ur_moveit_config") + "/srdf/ur.srdf.xacro",
                mappings={
                    "name": "ur",
                    "prefix": prefix
                }
            )
            .robot_description_kinematics("config/kinematics.yaml")
            .joint_limits("config/joint_limits.yaml")
            .trajectory_execution("config/moveit_controllers.yaml")
            .moveit_cpp("config/notebook.yaml")
            .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
            .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
            .pilz_cartesian_limits("config/pilz_cartesian_limits.yaml")
            .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'rviz', 'view_robot.rviz'])
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name="rviz2_moveit",
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # moveit_config.joint_limits,
        ],
    )

    # Servo node for realtime control
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
        output="screen",
    )

    nodes = [
        ur_type_arg,
        use_fake_hardware_arg,
        safety_limits_arg,
        safety_pos_margin_arg,
        safety_k_position_arg,
        description_package_arg,
        description_file_arg,
        moveit_config_package_arg,
        moveit_config_file_arg,
        warehouse_sqlite_path_arg,
        use_sim_time_arg,
        prefix_arg,
        launch_rviz_arg,
        launch_servo_arg,
        move_group_node,
        servo_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)

if __name__ == '__main__':
    generate_launch_description()