import launch
import launch_ros.actions
from launch.actions import (
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnProcessExit,
)

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
        parameters=[{
            'use_sim_time': True,
        }],
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