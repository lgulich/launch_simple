import os
import pathlib
from typing import Dict, List, Callable, Optional

from ament_index_python import get_package_share_directory
from launch_simple.all_types import *

# from launch import LaunchDescription
# from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
#                             IncludeLaunchDescription)
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch_ros.actions import (ComposableNodeContainer, LoadComposableNodes, Node,
#                                 PushRosNamespace, SetRemap)
# from launch_ros.descriptions import ComposableNode
# from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


class DotDict:
    """Dict-like class, that allows access to dict items with the dot notation."""

    def set(self, name, value):
        """Add a new key-value pair. The key is not allowed to exist yet."""
        assert not hasattr(self, name), f"Item with name '{name}' already exists."
        setattr(self, name, value)
        return value

    def keys(self):
        """Get all keys in the dict."""
        return self.__dict__.keys()

    def values(self):
        """Get all values in the dict."""
        return self.__dict__.values()

    def items(self):
        """Get tuples of keys and values in the dict."""
        return self.__dict__.items()

    def __len__(self):
        """Get number of items in dict."""
        return self.__dict__.__len__()

    def __getitem__(self, key):
        """Access a value by its key."""
        return getattr(self, key)


class Context:
    """Context used to build a launch_simple launch description."""

    def __init__(self):
        self.arguments = DotDict()

        self.actions = DotDict()
        self.composable_nodes = DotDict()

    def add_argument(
            self,
            name: str,
            type: Optional[Callable] = None,  #pylint: disable=redefined-builtin,unused-argument
            default: Optional[str] = None,
            description: Optional[str] = None,
            choices: Optional[str] = None,
            **kwargs):
        """ Add a launch argument to the launch description. """
        arg = DeclareLaunchArgument(
            name,
            default_value=default,
            description=description,
            choices=choices,
            **kwargs,
        )
        self.add_action(arg)

        value = LaunchConfiguration(name)
        return self.arguments.set(name, value)

    def add_action(self, action):
        """ Add any existing launch action to the launch description. """
        name = None
        if hasattr(action, 'name'):
            name = action.name
        if name is None:
            name = f'action_{len(self.actions)}'
        self.actions.set(name, action)
        return action

    def add_composable_node(self, name, package, plugin, **kwargs):
        """ Add a composable node. Nodes have to be added later to a composable node container. """
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
        """
        Add a container for composable nodes. Composable nodes can be added now, or later with
        'Context.add_to_container()'.
        """
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
        """ Add a composable node to a composable node container. """
        load = LoadComposableNodes(
            target_container=container,
            composable_node_descriptions=composable_nodes,
        )
        return self.add_action(load)

    def include_launch_file(
        self,
        path_or_package: str,
        package_relative_path: str = None,
        file_type: str = 'auto',
        namespace: str = '',
        launch_arguments: Dict = {},
        remappings: Dict[str, str] = {},
    ):
        """
        Include another launch file of any type into the launch context.

        Args:
            path_or_package(str): Path of the launch file or the package that contains it. If this
                                  is the path the 'package_relative_path' has to be 'None'.
            package_relative_path(str): Path of the launch file relative to it's package. Should
                                        only be set if 'path_or_package' is set to a package.
            file_type(str): File type of the included launch file. If set to 'auto' this will be
                            inferred automatically.
            namespace(str): If set the included launch file will be pushed to this namespace.
            launch_arguments(Dict): Launch arguments for the included launch file.
            remappings(Dict[str,str]): Topic remappings that will be applied to the included launch
                                       file.
        """
        if os.path.isfile(path_or_package):
            # path_or_package is already a valid path, thus we use this directly.
            assert package_relative_path is None, (
                f"Path was already set in 'path_or_package'(={path_or_package}). Please don't " +
                f"set 'package_relative_path'(={package_relative_path}) or set it to 'None'.")
            launch_file_path = path_or_package
        else:
            # Both path_or_package and package_relative_path were set, thus we have a package and a
            # relative path and we resolve the path accordingly
            assert package_relative_path is not None, (
                f"'path_or_package'(={path_or_package}) was set to package. Please set " +
                "'package_relative_path'.")
            package_path = get_package_share_directory(path_or_package)
            launch_file_path = os.path.join(package_path, package_relative_path)

        # Resolve launch file type by looking at file path extension.
        if file_type == 'auto':
            file_type = pathlib.Path(launch_file_path).suffix[1:]

        if file_type == 'py':
            launch_file_source = PythonLaunchDescriptionSource
        elif file_type == 'xml':
            launch_file_source = XMLLaunchDescriptionSource
        else:
            raise f"Unknown include file type '{file_type}'"

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

    def get_launch_description(self) -> LaunchDescription:
        """ Use this method to convert launch_simple to a default launch description. """
        all_actions = list(self.actions.values())
        return LaunchDescription(all_actions)


def unsimplify(generate_simple_launch_description: Callable[[Context], None]) -> LaunchDescription:
    """
    Convert a function with a simple launch description to the default launch description.
    """
    ctx = Context()
    generate_simple_launch_description(ctx)
    return ctx.get_launch_description()
