#!/bin/bash

# Use this script to launch a shell in a dev container.

set -e

script_dir=$(cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd)
cd "$script_dir"

docker build --file .devcontainer/Dockerfile --tag launch_simple_dev:latest .
docker run -it \
    --network=host \
    -v "$script_dir:/workspace/src/launch_simple" \
    launch_simple_dev:latest
