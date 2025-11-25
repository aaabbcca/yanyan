from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로 가져오기
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    urdf_file = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'turtlebot3_with_sensors.urdf'
    )
    
    urdf_content = open(urdf_file).read()
    
    # World 파일을 파라미터로 받기
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(gazebo_ros_pkg, 'worlds', 'empty.world'),
        description='Gazebo world file path'
    )
    
    # Gazebo 통합 실행 (이게 핵심!)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    return LaunchDescription([
        world_arg,
        gazebo,
        
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{'robot_description': urdf_content}]
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file, '-x', '8', '-y', '0', '-z', '0.5'],
            output='screen'
        ),
        
        # 3D LiDAR → 2D LaserScan 변환
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                    'target_frame': 'base_scan',
                    'transform_tolerance': 0.01,
                    'min_height': 0.15,
                    'max_height': 1.8,
                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.0087,
                    'scan_time': 0.1,
                    'range_min': 0.3,
                    'range_max': 30.0,
                    'use_inf': True,
                    'inf_epsilon': 1.0
            }],
            remappings=[
                ('cloud_in', '/velodyne_points'),
                ('scan', '/scan_filtered')
            ],
            output='screen'
        )
    ])