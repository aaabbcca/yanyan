import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = FindPackageShare('my_robot').find('my_robot')
    
    # World 파일 경로
    world_file = os.path.join(pkg_share, 'worlds', 'gochang_dolmen_park.world')
    
    # ⭐ HYOSUN의 SLAM 파라미터 사용!
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    
    # RViz 설정 파일
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'slam_visualization.rviz')
    
    # Gazebo 모델 경로 설정
    model_path = os.path.join(pkg_share, 'models')
    os.environ['GAZEBO_MODEL_PATH'] = model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Gazebo + 로봇 스폰
    gazebo_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'gazebo_with_robot.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
        }.items()
    )
    
    # 2. SLAM Toolbox (HYOSUN 설정 사용!)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # 3. RViz (3초 후 실행)
    rviz_delayed = TimerAction(
        period=3.0,
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
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo_robot,
        slam_toolbox,
        rviz_delayed
    ])
