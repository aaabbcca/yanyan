import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    pkg_share = FindPackageShare('my_robot').find('my_robot')
    
    # World 파일 경로
    world_file = os.path.join(pkg_share, 'worlds', 'gochang_dolmen_park.world')
    
    # SLAM 파라미터 파일 (있으면 사용, 없으면 기본값)
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    if not os.path.exists(slam_params_file):
        slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    # RViz 설정 파일
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_visualization.rviz')
    
    # Gazebo 모델 경로 설정
    model_path = os.path.join(pkg_share, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ========== 1. Gazebo + 로봇 ==========
    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo_with_robot.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
        }.items()
    )
    
    # ========== 2. SLAM Toolbox ==========
    slam_toolbox = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': use_sim_time}
                ],
                output='screen'
            )
        ]
    )
    
    # ========== 3. RViz ==========
    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_file],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    # ========== 4. 카메라 뷰어 (rqt_image_view) ==========
    camera_view = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['xterm', '-e', 'bash', '-c', 
                     'source /opt/ros/humble/setup.bash && '
                     'source ' + os.path.expanduser('~/yanyan/install/setup.bash') + ' && '
                     'ros2 run rqt_image_view rqt_image_view /camera_center/camera_center/image_raw; '
                     'exec bash'],
                output='screen',
                shell=False
            )
        ]
    )
    
    # ========== 5. Teleop 키보드 제어 ==========
    teleop = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['xterm', '-hold', '-e', 'bash', '-c',
                     'source /opt/ros/humble/setup.bash && '
                     'source ' + os.path.expanduser('~/yanyan/install/setup.bash') + ' && '
                     'echo "========================================" && '
                     'echo "🎮 로봇 키보드 제어" && '
                     'echo "========================================" && '
                     'echo "  w/x : 전진/후진" && '
                     'echo "  a/d : 좌회전/우회전" && '
                     'echo "  s   : 정지" && '
                     'echo "  q/e : 속도 증가/감소" && '
                     'echo "========================================" && '
                     'echo "" && '
                     'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
                output='screen',
                shell=False
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_robot,
        slam_toolbox,
        rviz,
        camera_view,
        teleop
    ])
