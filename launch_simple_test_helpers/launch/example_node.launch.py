from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """ Launch an example node. """
    return LaunchDescription([
        DeclareLaunchArgument('message_content', default_value='default_message_content'),
        Node(
            name='example_node',
            package='launch_simple_test_helpers',
            executable='example_node',
            parameters=[{
                'message_content': LaunchConfiguration('message_content')
            }],
        ),
    ])
