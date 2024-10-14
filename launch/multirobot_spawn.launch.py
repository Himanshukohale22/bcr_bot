import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
import xacro

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():

    bcr_bot_path = get_package_share_directory('bcr_bot')

    xacro_path = os.path.join(bcr_bot_path,'urdf','bcr_bot.xacro')

        # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path
                    ])}],

    )

    robot1_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='bcr_bot1',
        output='screen',
        arguments=[
            '-topic','/robot_description',
            '-entity',PythonExpression(['"', 'bcr_bot1','_robot"']),
            '-z', "0.28",
            '-x', '0.0',
            '-y', '0.0',
            '-Y', '0.0'
        ]
    )

    robot2_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='bcr_bot2',
        output='screen',
        arguments=[
            '-topic','/robot_description',
            '-entity',PythonExpression(['"', 'bcr_bot2','_robot"']),
            '-z', "0.28",
            '-x', '0.0',
            '-y', '0.2',
            '-Y', '0.0'
        ]
    )

    return LaunchDescription([

        robot_state_publisher,
        robot1_spawn_entity,
        robot2_spawn_entity

    ])


