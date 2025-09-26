from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fc_comm',
            executable='guidance_simulator',
            name='guidance_simulator',
            output='screen'
        ),
        Node(
            package='fc_comm',
            executable='fc_comm_node',
            name='fc_comm_node',
            output='screen'
        )
    ])