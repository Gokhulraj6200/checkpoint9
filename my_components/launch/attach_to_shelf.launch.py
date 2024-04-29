import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Main package
    pkg = get_package_share_directory('my_components')

    # Pre-approach node configuration
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'),
            ],
            output='screen',
    )

    manual_node = Node(
            package='my_components',
            executable='manual_composition',
            output='screen',
            name='manual_composition')

    return LaunchDescription([
        container,
        manual_node
    ])