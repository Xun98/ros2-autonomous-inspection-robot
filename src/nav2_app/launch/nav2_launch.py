import launch
import launch_ros.actions
from launch.actions import (
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessExit,
)

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

goals_config = PathJoinSubstitution([FindPackageShare('nav2_app'), 'config', 'goals.yaml']) 

init_node = launch_ros.actions.Node(
        package='nav2_app',
        executable='initpose_pub',
        name='initpose_publisher',
        parameters=[{
            'use_sim_time': True,
        }],
    )

goal_node = launch_ros.actions.Node(
        package='nav2_app',
        executable='to_goal',
        name='nav2_action_client',
        parameters=[
            {'use_sim_time': True},
            goals_config
        ],
    )

def generate_launch_description():
    return launch.LaunchDescription([
        init_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=init_node,
                on_exit=[
                    goal_node
                ]
            )
        ),
    ])