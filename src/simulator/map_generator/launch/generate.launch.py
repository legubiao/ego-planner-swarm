from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_generator',
            executable='random_forest',
            name='random_forest',
            output='screen',
            parameters=[
                {'map/x_size': 36.0},
                {'map/y_size': 20.0},
                {'map/z_size': 3.0},
                {'map/resolution': 0.1},
                {'ObstacleShape/seed': 1},
                {'map/obs_num': 200},
                {'ObstacleShape/lower_rad': 0.5},
                {'ObstacleShape/upper_rad': 0.7},
                {'ObstacleShape/lower_hei': 0.0},
                {'ObstacleShape/upper_hei': 3.0},
                {'map/circle_num': 200},
                {'ObstacleShape/radius_l': 0.7},
                {'ObstacleShape/radius_h': 0.5},
                {'ObstacleShape/z_l': 0.7},
                {'ObstacleShape/z_h': 0.8},
                {'ObstacleShape/theta': 0.5},
                {'sensing/radius': 5.0},
                {'sensing/rate': 1.0},
                {'min_distance': 1.2},
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', 'install/map_generator/share/map_generator/rviz/map.rviz']
        )
    ])