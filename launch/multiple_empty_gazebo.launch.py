import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    spawn_multiple_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bcr_bot_path,'launch','multirobot_spawn.launch.py'))
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(bcr_bot_path, "models")),

        gazebo,
        spawn_multiple_bot

    ])
