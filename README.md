[![CI](https://github.com/lgulich/launch_simple/actions/workflows/ci.yml/badge.svg)](https://github.com/lgulich/launch_simple/actions/workflows/ci.yml)
[![Pre-commit Dependency Update](https://github.com/lgulich/launch_simple/actions/workflows/pre_commit_dependency_update.yml/badge.svg)](https://github.com/lgulich/launch_simple/actions/workflows/pre_commit_dependency_update.yml)

# Launch Simple
A tool to simplify ROS2 launch files.

## Features
- [x] Simpler API
  - [x] Declare launch entities only once
    (-> no more assign-entity-to-variable and add-variable-to-list)
  - [x] Import all needed types in a single import statement
    (-> no more "which import am I missing?")
  - [x] Dict's used wherever possible
    (-> no more `{"key": "value"}.items()` statements)
  - [x] Argparse-like launch arguments declaration
    (-> no more duplication between `DeclareLaunchArgument` and `LaunchConfiguration`)
- [x] Simpler launch file nesting
  - [x] Include existing py/xml launch files in `launch_simple` and vice versa
    (-> fully back and forward comaptible)
  - [x] Include launch files by package
    (-> no more manually declared package share directories)
  - [x] Easily add namespace/remappings for included launch files
    (-> no more `GroupActions` required)
- [ ] Default configurations for all nodes:
    - [ ] Set default `output` for all nodes
    - [ ] Set default `event_handler`s for all nodes
      (-> fail the launch file if a node fails.)

## Getting Started

Getting started is simple. Just clone this repo into your colcon workspace and build it with colcon.

You can clone with one of the following options:
- Using the git CLI? Run this command in your terminal:
  ```sh
  git clone https://github.com/lgulich/launch_simple.git
  ```
- Using the [VCS tool](https://github.com/dirk-thomas/vcstool)? Add the following to your versions
  file:
  ```yaml
  repositories:
    lgulich/launch_simple:
      type: git
      url: https://github.com/lgulich/launch_simple.git
      version: master
  ```

## API Example
For an extended example see
[example_simple.launch.py](launch_simple/launch/example_simple.launch.py)
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

## Documentation
Visit the documentation [here](https://lgulich.github.io/launch_simple/)
