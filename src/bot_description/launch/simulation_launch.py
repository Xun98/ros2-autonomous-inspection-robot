from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer

def generate_launch_description():
    pkg_share = FindPackageShare('bot_description')
    model_path = PathJoinSubstitution([pkg_share, 'urdf', 'bot_description.sdf'])
    world_path = PathJoinSubstitution([pkg_share, 'urdf', 'my_world.sdf'])
    
    bridge_config_path = PathJoinSubstitution([pkg_share, 'config', 'bridge_config.yaml'])
    
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')
    gz_spawn_model_launch_source = PathJoinSubstitution([ros_gz_sim_share, "launch", "gz_spawn_model.launch.py"])



    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    
    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container='False',
        use_composition='True',
    )

    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container='True',
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
    
    
    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=model_path, description='Absolute path to robot model file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='log'),
        gz_server,
        spawn_entity,
        ros_gz_bridge
    ])