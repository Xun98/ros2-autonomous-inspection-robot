from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    pkg_share = FindPackageShare('nav2_server')
    bridge_config_path = PathJoinSubstitution([pkg_share, 'config', 'bridge_config.yaml'])
    nav2_param_path = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    room_path = PathJoinSubstitution([pkg_share, 'mapInfo', 'room.yaml'])

    nav2_bringup_share = FindPackageShare('nav2_bringup')
    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    nav2_launch_file = PathJoinSubstitution([
        nav2_bringup_share,
        'launch',
        'bringup_launch.py'
    ])
        
    nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': LaunchConfiguration('room'),
            'params_file': LaunchConfiguration('nav2_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'True',
            'slam': 'False',
            'use_composition': 'False',
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='room', default_value=room_path, description='map'),
        DeclareLaunchArgument(name='nav2_config', default_value=nav2_param_path, description='nav2 config'),

        robot_localization_node,
        nav2_bringup,
    ])