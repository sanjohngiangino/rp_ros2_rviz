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
                'map_path': '/home/john/Desktop/rp_ros2_rviz/map/cappero_laser_odom_diag_2020-05-06-16-26-03.png'
            }]
        ),
    ])
#ros2 run rp_ros2_rviz mapserver --ros-args -p map_path:=/home/john/Desktop/rp_ros2_rviz/map/labirinto.png