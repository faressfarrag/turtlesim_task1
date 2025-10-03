from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch turtlesim simulator
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Launch your custom controller
        Node(
            package='my_turtle_pkg',
            executable='turtle_autonomy',
            name='controller',
            output='screen'
        )
    ])
