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


    # robot1_position_x = LaunchConfiguration("robot1_position_x")
    # robto1_position_y = LaunchConfiguration("robot1_position_y")
    # robot1_orientation_yaw = LaunchConfiguration("robot1_orientation_yaw")


    # robot2_position_x = LaunchConfiguration("robot2_position_x")
    # robto2_position_y = LaunchConfiguration("robot2_position_y")
    # robot2_orientation_yaw = LaunchConfiguration("robot2_orientation_yaw")


    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")

    
    # Launch the robot_state_publisher node
    robot1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='bcr_bot1',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,                   
                     ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', 'bcr_bot1',
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', 'bcr_bot1', '/joint_states"'])),
        ]

    )

    robot2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='bcr_bot2',
        output='screen',
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', xacro_path,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', 'bcr_bot2',
                    ])}],
        remappings=[
            ('/joint_states', PythonExpression(['"', 'bcr_bot2', '/joint_states"'])),
        ]

    )

    robot1_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='bcr_bot1',
        output='screen',
        arguments=[
            '-topic','/bcr_bot1/robot_description',
            '-entity',PythonExpression(['"', 'bcr_bot1','_robot"']),
            '-z', "0.0077",
            '-x', '0.05105',
            '-y', '0.02532',
            '-Y', '0.997'
        ]
    )

    robot2_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='bcr_bot2',
        output='screen',
        arguments=[
            '-topic','/bcr_bot2/robot_description',
            '-entity',PythonExpression(['"', 'bcr_bot2','_robot"']),
            '-z', "-0.0539",
            '-x', '1.0883',
            '-y', '-5.605',
            '-Y', '-0.5393'
        ]
    )

    return LaunchDescription([
        
    #     DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
    #     DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
    #     DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
    #     DeclareLaunchArgument("odometry_source", default_value = odometry_source),

        robot1_state_publisher,
        robot2_state_publisher,
        robot1_spawn_entity,
        robot2_spawn_entity

    ])


