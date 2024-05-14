import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    ur_moveit_launch_dir = get_package_share_directory('ur_moveit_config')

    left_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', 'ur_moveit.launch.py')))
    
    left_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('left'),
         left_moveit
      ]
    )

    right_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_moveit_launch_dir, 'launch', 'ur_moveit.launch.py')))
    
    right_moveit_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('right'),
         right_moveit
      ]
    )

    
    return LaunchDescription([
        left_moveit_with_namespace,
        right_moveit_with_namespace
    ])
