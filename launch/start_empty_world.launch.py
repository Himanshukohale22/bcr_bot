from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_bc_bot = get_package_share_directory('bcr_bot')

    world_file = LaunchConfiguration("world_file", default = join(pkg_bc_bot, 'worlds', 'small_warehouse.sdf'))

    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    return LaunchDescription([

        SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value="/usr/share/gazebo-11:" + join(pkg_bc_bot, "worlds")),
        DeclareLaunchArgument('world', default_value = world_file),
        gazebo
    ])



