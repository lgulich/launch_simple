import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Add a parameter.
    chatter_namespace_arg = DeclareLaunchArgument('chatter_namespace', default_value='chatter_ns')
    chatter_namespace_value = LaunchConfiguration('chatter_namespace')

    # Add a classic node.
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='sim',
        namespace='turtlesim_ns',
    )

    # Add an executable.
    rosbag_play = ExecuteProcess(cmd=['echo', 'Hello World!'])

    # Add a composable node and node container in a single step.
    composable_node_a = ComposableNode(
        name='composable_node_a',
        package='composition',
        plugin='composition::Talker',
    )
    container_a = ComposableNodeContainer(
        name='container_a',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[composable_node_a],
    )

    # Add a composable node and node container in two steps.
    composable_node_b = ComposableNode(
        name='composable_node_b',
        package='composition',
        plugin='composition::Talker',
    )
    container_b = Node(
        name='container_b',
        package='rclcpp_components',
        executable='component_container',
    )
    load_container_b = LoadComposableNodes(
        target_container='container_b',
        composable_node_descriptions=[composable_node_b],
    )

    # # Include an external launch file, either python, xml.
    launch_chatter_xml = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('demo_nodes_cpp'),
                         'launch/topics/talker_listener.launch.xml')))

    # Include another launch file in the chatter_ns namespace
    launch_chatter_with_args = GroupAction(actions=[
        PushRosNamespace(chatter_namespace_value),
        SetRemap(src='chatter', dst='other_chatter'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('demo_nodes_cpp'),
                             'launch/topics/talker_listener.launch.py')),
            launch_arguments={'dummy': 'dummy_value'}.items(),
        ),
    ])

    return LaunchDescription([
        chatter_namespace_arg,
        turtlesim,
        rosbag_play,
        container_a,
        container_b,
        load_container_b,
        launch_chatter_xml,
        launch_chatter_with_args,
    ])
