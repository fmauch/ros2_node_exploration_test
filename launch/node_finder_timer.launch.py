from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    example_node = Node(
        package='examples_rclpy_minimal_publisher',
        executable='publisher_local_function',
        name='talker',
    )
    exploration_nodes = [
        Node(
            package='node_exploration',
            executable='node_finder_timer',
            name=f'node_finder_{i}',
            output='screen',
        )
        for i in range(50)
    ]

    return LaunchDescription([example_node] + exploration_nodes)
