from launch_simple.all_types import *
import launch_simple as ls


def generate_launch_description():
    """ Test a launch file with arguments. """
    ctx = ls.Context()

    # Add arguments.
    ctx.add_argument('message_content', type=str, default='my_example_message')
    ctx.add_argument('publish_count', type=int, default=10)
    ctx.add_argument('publish_interval', type=float, default=0.1)
    ctx.add_argument('enable_node', type=bool, default=True)

    # Pass arguments to a node
    ctx.add_action(
        Node(name='example_node',
             package='launch_simple_test_helpers',
             executable='example_node',
             namespace='example_node_namespace',
             parameters=[{
                 'message_content': ctx.arguments.message_content,
                 'publish_count': ctx.arguments.publish_count,
                 'publish_interval': ctx.arguments.publish_interval,
             }],
             condition=IfCondition(ctx.arguments.enable_node),
             output='both'),)

    return ctx.get_launch_description()
