import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():

    bcr_bot_path = get_package_share_directory('bcr_bot')

    xacro_path = os.path.join(bcr_bot_path,'urdf','bcr_bot.xacro')

    camera_enable = LaunchConfiguration("camera_enable",default=True)
    stereo_camera_enable = LaunchConfiguration("stereo_camera_enambled",default=True)
    two_d_lidar_enable = LaunchConfiguration("two_d_lidar",default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot1_namespace = LaunchConfiguration("robot1_namespace",default='bcr_bot1')
    robot2_namespace = LaunchConfiguration("robot1_namespace",default='bcr_bot2')

    # Launch the robot_state_publisher node

    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot1_namespace,
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                     ' camera_enabled:=', camera_enable,
                    ' stereo_camera_enabled:=', stereo_camera_enable,
                    ' two_d_lidar_enabled:=', two_d_lidar_enable,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,                  
                    ' robot_namespace:=', robot1_namespace,
                    ])}],
        remappings=[
            ('/joint_states', 'bcr_bot1/joint_states')
        ]

    )

    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot2_namespace,
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                     ' camera_enabled:=', camera_enable,
                    ' stereo_camera_enabled:=', stereo_camera_enable,
                    ' two_d_lidar_enabled:=', two_d_lidar_enable,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source, 
                    ' robot_namespace:=', robot2_namespace,
                    ])}],
        remappings=[
            ('/joint_states', 'bcr_bot2/joint_states')
        ]

    )

    robot1_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=robot1_namespace,
        output='screen',
        arguments=[
            '-topic','/bcr_bot1/robot_description',
            '-entity',f'/{robot2_namespace}/robot_description',
            '-z', "0.0",
            '-x', '0.0',
            '-y', '0.5',
            '-Y', '0.0'
        ]
    )

    robot2_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=robot2_namespace,
        output='screen',
        arguments=[
            '-topic','/bcr_bot2/robot_description',
            '-entity', f'/{robot2_namespace}/robot_description',
            '-z', "0.0",
            '-x', '0.0',
            '-y', '-0.9',
            '-Y', '0.0'
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument("camera_enabled", default_value = camera_enable),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enable),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enable),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),

        robot1_state_publisher,
        robot1_spawn_entity,
        robot2_state_publisher,
        robot2_spawn_entity

    ])


