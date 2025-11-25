import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Navigation 노드들
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )
    
    # Lifecycle Manager (자동 활성화)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0
        }]
    )
    
    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        # Lifecycle Manager를 5초 후에 시작 (노드들이 준비될 시간 확보)
        TimerAction(
            period=5.0,
            actions=[lifecycle_manager]
        )
    ])