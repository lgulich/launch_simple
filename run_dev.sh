#!/bin/bash

# Use this script to launch a shell in a dev container.

set -e

script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
cd "$script_dir"

devcontainer up --workspace-folder . --remove-existing-container
devcontainer exec --workspace-folder . bash
