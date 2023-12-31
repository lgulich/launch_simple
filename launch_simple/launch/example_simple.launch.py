import launch_simple as ls
from launch_simple.all_types import *


def generate_launch_description():
    """
    Example of a fairly complicated launch description using the launch_simple syntax.
    """
    ctx = ls.Context()
    # Add a parameter.
    ctx.add_argument('chatter_namespace', type=str, default='chatter_ns')

    # Add a classic node.
    ctx.add_action(
        Node(name='talker', package='demo_nodes_cpp', executable='talker', namespace='talker_ns'))

    # Add an executable.
    ctx.add_action(ExecuteProcess(cmd=['sleep', '10s'], on_exit=Shutdown()))

    # Add a composable node and node container in a single step.
    ctx.add_composable_node(
        name='composable_node_a',
        package='composition',
        plugin='composition::Talker',
    )
    ctx.add_composable_node_container(
        name='container_a',
        package='rclcpp_components',
        executable='component_container',
        composable_nodes=[ctx.composable_nodes.composable_node_a],
    )

    # Add a composable node and node container in two steps.
    ctx.add_composable_node(
        name='composable_node_b',
        package='composition',
        plugin='composition::Talker',
    )
    ctx.add_composable_node_container(
        name='container_b',
        package='rclcpp_components',
        executable='component_container',
    )
    ctx.add_to_container('container_b', [ctx.composable_nodes.composable_node_b])

    # Include an external launch file, either python, xml.
    ctx.include_launch_file('demo_nodes_cpp', 'launch/topics/talker_listener.launch.xml')

    # Pass parameters/remappings to a launch file.
    ctx.include_launch_file(
        'demo_nodes_cpp',
        'launch/topics/talker_listener.launch.xml',
        launch_arguments={'dummy': 'dummy_value'},
        namespace=ctx.arguments.chatter_namespace,
        remappings={
            'chatter': 'other_chatter',
        },
    )
    return ctx.get_launch_description()
