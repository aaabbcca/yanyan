from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Octomap Server
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},  # 5cm 해상도
            {'frame_id': 'map'},
            {'sensor_model/max_range': 25.0},
            {'filter_ground': False},
            {'height_map': False},
            {'color/use_rgb': True}
        ],
        remappings=[
            ('cloud_in', '/velodyne_points')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        octomap_server
    ])
