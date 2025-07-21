from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

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
        Node(
            package='rp_ros2_rviz',
            executable='particle_node_filter',
            name='particle_node_filter',
            output='screen'
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '-1', '/initialpose', 'geometry_msgs/Pose',
                        "{\"position\":{\"x\":10.0,\"y\":10.0,\"z\":0.0},"
                        "\"orientation\":{\"x\":0.0,\"y\":0.0,\"z\":0.0,\"w\":1.0}}"
                    ],
            
                    output='screen'
                )
            ]
        ),
    ])
#ros2 run rp_ros2_rviz mapserver --ros-args -p map_path:=/home/john/Desktop/rp_ros2_rviz/map/labirinto.png

"""
ros2 topic pub --once /initialpose geometry_msgs/Pose "
position:
  x: 300.0
  y: 200.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
"

ros2 topic pub --once /move_base/goal geometry_msgs/Point "{x: 300.0, y: 200.0, z: 0.0}"

"""