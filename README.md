# Launch Simple
[![CI](https://github.com/lgulich/launch_simple/actions/workflows/ci.yml/badge.svg)](https://github.com/lgulich/launch_simple/actions/workflows/ci.yml)
[![Pre-commit Dependency Update](https://github.com/lgulich/launch_simple/actions/workflows/pre_commit_dependency_update.yml/badge.svg)](https://github.com/lgulich/launch_simple/actions/workflows/pre_commit_dependency_update.yml)

A tool to simplify ROS2 launch files.

## Features
- [x] Declare items only once (ie. no return of used items)
- [x] Include default launch files in `launch_simple`
- [x] Include `launch_simple` in default launch files
- [x] Argparse like arguments declaration
- [x] Include other launch files by package
- [x] Import all needed types in 1 command
- [x] No weird lists of “pairs”, dicts are used wherever possible
- [x] Easily remap topics in included launch files, no "GroupActions" required
- [ ] Set default configurations:
    - [ ] Set output for all nodes
    - [ ] Set all nodes to 'required', ie. fail the launch file if a node fails.

## Example
For an extended example see [simple.launch.py](launch_simple/launch/simple.launch.py)
```py
import launch_simple as ls
from launch_simple.all_types import *

def generate_launch_simple_description():
    # Create the launch context.
    ctx = ls.Context()

    # Add a launch argument
    ctx.add_argument('chatter_namespace', type=str, default='chatter_ns')

    # Include another launch file and pass arguments, add a namespace and remap topics.
    ctx.include_launch_file(
        'demo_nodes_cpp',
        'launch/topics/talker_listener.launch.xml',
        launch_arguments={'dummy': 'dummy_value'},
        namespace=ctx.arguments.chatter_namespace,
        remappings={
            'chatter': 'other_chatter',
        },
    )

    # Convert the launch_simple context to a default launch_description.
    return ctx.get_launch_description()
```
