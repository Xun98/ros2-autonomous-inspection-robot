from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

use_sim_time = LaunchConfiguration('use_sim_time')
def generate_launch_description():
    pkg_share = FindPackageShare('bot_bringup')
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])
    
    nav2_app_share = FindPackageShare('nav2_app')
    nav2_app_file = PathJoinSubstitution([
        nav2_app_share,
        'launch',
        'nav2_launch.py'
    ])
    
    nav2_server_share = FindPackageShare('nav2_server')
    nav2_server_file = PathJoinSubstitution([
        nav2_server_share,
        'launch',
        'server_launch.py'
    ])
    
    sim_share = FindPackageShare('bot_description')
    sim_file = PathJoinSubstitution([
        sim_share,
        'launch',
        'simulation_launch.py'
    ])
    
    sim = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(sim_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
    )
    
    nav2_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_server_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
        
    
    nav2_app = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(nav2_app_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Flag to enable use_sim_time'),
        
        sim,
        rviz_node,
        nav2_server,
        nav2_app
    ])