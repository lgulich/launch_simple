#!/bin/bash

# Use this script to test everything.

run_in_devcontainer() {
  devcontainer exec --workspace-folder . $@
}

set -e

script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
cd "$script_dir"

devcontainer up --workspace-folder . --remove-existing-container

run_in_devcontainer colcon build
run_in_devcontainer make -C src/launch_simple/doc html
run_in_devcontainer colcon test
run_in_devcontainer ros2 launch launch_simple example_default.launch.py
run_in_devcontainer ros2 launch launch_simple example_simple.launch.py
run_in_devcontainer ros2 launch launch_simple test_arguments.launch.py
run_in_devcontainer ros2 launch launch_simple test_composable_node.launch.py
run_in_devcontainer ros2 launch launch_simple test_includes.launch.py
