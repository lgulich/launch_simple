import os
import pathlib
from typing import Dict, List

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, PushRosNamespace, SetRemap
from launch_ros.descriptions import ComposableNode


def unsimplify(simple_generator) -> LaunchDescription:
    ctx = Context()
    simple_generator(ctx)
    return ctx._get_launch_description()


class DotDict:

    def set(self, name, value):
        assert not hasattr(self, name), f"Item with name '{name}' already exists."
        setattr(self, name, value)
        return value

    def keys(self):
        return self.__dict__.keys()

    def values(self):
        return self.__dict__.values()

    def items(self):
        return self.__dict__.items()

    def __len__(self):
        return self.__dict__.__len__()


class Context:

    def __init__(self):
        self.params = DotDict()

        self.actions = DotDict()
        self.composable_nodes = DotDict()

    def add_parameter(self, name: str, type=None, default=None):
        arg = DeclareLaunchArgument(name, default_value=default)
        self.add_action(arg)

        value = LaunchConfiguration(name)
        return self.params.set(name, value)

    def add_action(self, action):
        name = None
        if hasattr(action, 'name'):
            name = action.name
        if name is None:
            name = f'action_{len(self.actions)}'
        self.actions.set(name, action)
        return action

    def add_composable_node(self, name, package, plugin, **kwargs):
        node = ComposableNode(
            name=name,
            package=package,
            plugin=plugin,
            **kwargs,
        )
        return self.composable_nodes.set(name, node)

    def add_composable_node_container(self,
                                      name,
                                      package,
                                      executable,
                                      namespace='',
                                      composable_nodes=[],
                                      **kwargs):
        container = ComposableNodeContainer(
            name=name,
            package=package,
            executable=executable,
            namespace=namespace,
            **kwargs,
            composable_node_descriptions=composable_nodes,
        )
        return self.add_action(container)

    def add_to_container(self, container: str, composable_nodes: List[ComposableNode]):
        load = LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=composable_nodes,
        )
        return self.add_action(load)

    def include_launch_file(
        self,
        path_or_package: str,
        package_relative_path: str = None,
        type: str = 'auto',
        namespace: str = '',
        launch_arguments: Dict = {},
        remappings: Dict[str, str] = {},
    ):
        if package_relative_path:
            # Both launch_file and launch_file_or_package were set, thus we were passed package and
            # path.
            launch_file_path = os.path.join(get_package_share_directory(path_or_package),
                                            package_relative_path)
        else:
            launch_file_path = path_or_package

        # Resolve launch file type by looking at file path extension.
        if type == 'auto':
            type = pathlib.Path(launch_file_path).suffix[1:]

        if type == 'py':
            launch_file_source = PythonLaunchDescriptionSource
        elif type == 'xml':
            launch_file_source = XMLLaunchDescriptionSource
        else:
            raise f"Unknown include type '{type}'"

        actions = []
        if namespace:
            actions.append(PushRosNamespace(namespace))
        if remappings:
            for src, dst in remappings.items():
                actions.append(SetRemap(src=src, dst=dst))

        actions.append(
            IncludeLaunchDescription(launch_file_source(launch_file_path),
                                     launch_arguments=launch_arguments.items()))
        launch = GroupAction(actions=actions)
        return self.add_action(launch)

    def _get_launch_description(self):
        all_actions = list(self.actions.values())
        return LaunchDescription(all_actions)
