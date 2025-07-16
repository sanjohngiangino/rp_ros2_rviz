from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rp_ros2_rviz',
            executable='world_node',
            name='world_node',
            output='screen'
        ),
        Node(
            package='rp_ros2_rviz',
            executable='controller_node',
            name='controller_node',
            output='screen'
        ),
        Node(
            package='rp_ros2_rviz',
            executable='pathplanner_node',
            name='pathplanner_node',
            output='screen'
        ),
        Node(
            package='rp_ros2_rviz',
            executable='mapserver',
            name='map_server',
            output='screen',
            parameters=[{
                'map_path': '/home/john/Desktop/rp_ros2_rviz/map/labirinto.png'
            }]
        ),
    ])
