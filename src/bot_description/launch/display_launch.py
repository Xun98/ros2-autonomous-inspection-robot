from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    pkg_share = FindPackageShare('bot_description')
    model_path = PathJoinSubstitution([pkg_share, 'urdf', 'bot_description.sdf'])
    world_path = PathJoinSubstitution([pkg_share, 'urdf', 'my_world.sdf'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])
    bridge_config_path = PathJoinSubstitution([pkg_share, 'config', 'bridge_config.yaml'])
    nav2_param_path = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    room_path = PathJoinSubstitution([pkg_share, 'mapInfo', 'room.yaml'])
    
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    gz_spawn_model_launch_source = PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"])
    
    nav2_bringup_share = FindPackageShare('nav2_bringup')

    nav2_launch_file = PathJoinSubstitution([
        nav2_bringup_share,
        'launch',
        'bringup_launch.py'
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
        use_composition='True',
    )
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )
    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'my_world',
            'topic': '/robot_description',
            'entity_name': 'bot',
            'z': '0.65',
        }.items(),
    )
    
    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml']), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
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
        DeclareLaunchArgument(name='model', default_value=model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path, description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='room', default_value=room_path, description='map'),
        DeclareLaunchArgument(name='nav2_config', default_value=nav2_param_path, description='nav2 config'),
        robot_state_publisher_node,
        robot_localization_node,
        
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        nav2_bringup,
        rviz_node,
    ])