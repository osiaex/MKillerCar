from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mosquito_car',
            executable='fixed_route_nav',
            name='fixed_route_nav',
            output='screen',
            parameters=[{'waypoints_file': '/home/ubuntu/ros2_ws/src/mosquito_car/waypoints.yaml'}]
        )
    ])
