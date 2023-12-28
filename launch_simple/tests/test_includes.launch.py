import launch_simple as ls
from launch_simple.all_types import *


def generate_launch_description():
    """ Test a launch file with other included launch files. """
    ctx = ls.Context()

    ctx.include_launch_file(
        'launch_simple_test_helpers',
        'launch/example_node.launch.xml',
        launch_arguments={'message_content': 'xml_message_content'},
        namespace='xml_launch_file_ns',
        remappings={'example_topic': 'xml_example_topic'},
    )

    ctx.include_launch_file(
        'launch_simple_test_helpers',
        'launch/example_node.launch.py',
        launch_arguments={'message_content': 'py_message_content'},
        namespace='py_launch_file_ns',
        remappings={'example_topic': 'py_example_topic'},
    )

    return ctx.get_launch_description()
