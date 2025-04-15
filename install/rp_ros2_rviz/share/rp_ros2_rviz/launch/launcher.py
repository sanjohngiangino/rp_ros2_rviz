from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_ros2_rviz',
            executable='map_server',
            name='map_server',
            parameters=[
                {'map_path': '/home/john/Desktop/rp_ros2_rviz/map/stanza.png'},
                {'resolution': 0.05},
                {'origin': [0.0, 0.0]}
            ]
        )
    ])
