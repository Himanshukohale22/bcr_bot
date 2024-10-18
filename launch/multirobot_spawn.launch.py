#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_bot_path = get_package_share_directory('bcr_bot')
    
    # Retrieve launch configuration arguments
    bcr_bot1_position_x = LaunchConfiguration("bcr_bot1_position_x")
    bcr_bot1_position_y = LaunchConfiguration("bcr_bot1_position_y")
    bcr_bot1_orientation_yaw = LaunchConfiguration("bcr_bot1_orientation_yaw")


    bcr_bot2_position_x = LaunchConfiguration("bcr_bot2_position_x")
    bcr_bot2_position_y = LaunchConfiguration("bcr_bot2_position_y")
    bcr_bot2_orientation_yaw = LaunchConfiguration("bcr_bot2_orientation_yaw")

    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot1_namespace = LaunchConfiguration("robot1_namespace", default='bcr_bot1')
    robot2_namespace = LaunchConfiguration("robot2_namespace", default='bcr_bot2')
    
    # Path to the Xacro file
    xacro_path = join(bcr_bot_path, 'urdf', 'bcr_bot.xacro')
    #doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true", "two_d_lidar_enabled": "true", "camera_enabled": "true"})

    # Launch the robot_state_publisher node
    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot1_namespace,
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot1_namespace, '/joint_states"'])),
        ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot1_namespace, '_robot"']), #default enitity name _bcr_bot
            '-z', "0.0",
            '-x', bcr_bot1_position_x,
            '-y', bcr_bot1_position_y,
            '-Y', bcr_bot1_orientation_yaw
        ]
    )

    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', robot1_namespace,
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', robot1_namespace, '/joint_states"'])),
        ]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', "/robot_description",
            '-entity', PythonExpression(['"', robot1_namespace, '_robot"']), #default enitity name _bcr_bot
            '-z', "0.0",
            '-x', bcr_bot2_position_x,
            '-y', bcr_bot2_position_y,
            '-Y', bcr_bot2_orientation_yaw
        ]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),

        DeclareLaunchArgument("bcr_bot1_position_x", default_value="0.0"),
        DeclareLaunchArgument("bcr_bot1_position_y", default_value="0.0"),
        DeclareLaunchArgument("bcr_bot1_orientation_yaw", default_value="0.0"),


        DeclareLaunchArgument("bcr_bot2_position_x", default_value="0.0"),
        DeclareLaunchArgument("bcr_bot2_position_y", default_value="-0.9"),
        DeclareLaunchArgument("bcr_bot2_orientation_yaw", default_value="0.0"),

        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        DeclareLaunchArgument("robot1_namespace", default_value = robot1_namespace),
        DeclareLaunchArgument("robot2_namespace", default_value = robot2_namespace),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),

        robot_state_publisher1,

        spawn_entity1,

        robot_state_publisher2,

        spawn_entity2
])
