import launch_simple as ls
from launch_simple.all_types import *


def generate_launch_description():
    """ Test a launch file with composable nodes. """
    ctx = ls.Context()

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

    # Shut down the launch file after 10s.
    ctx.add_action(ExecuteProcess(cmd=['sleep', '10s'], on_exit=Shutdown()))

    return ctx.get_launch_description()
