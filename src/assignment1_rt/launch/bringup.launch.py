import os

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('assignment1_rt')
    
    distance_params_file = os.path.join(pkg_share, 'config', 'distance_params.yaml')

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    distance_monitor_node = Node(
        package='assignment1_rt',
        executable='distance_monitor_node',
        name='distance_monitor',
        output='screen',
        parameters=[distance_params_file]
    )

    # Spawn turtle2 a bit after turtlesim starts
    spawn_turtle2 = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/spawn',
                    'turtlesim/srv/Spawn',
                    '{x: 5.0, y: 5.0, theta: 0.0, name: "turtle2"}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        turtlesim_node,
        distance_monitor_node,
        spawn_turtle2
    ])
