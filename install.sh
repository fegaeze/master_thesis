#!/bin/bash

# Get the directory of the currently executing script
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Define paths to library directories
unitree_legged_sdk_path="$workspace_dir/src/unitree_legged_sdk"

# Install system dependencies
sudo apt-get update
sudo apt-get install -y libzmq3-dev libboost-dev


# Build libraries
build_library() {
    local library_dir="$1"
    cd "$library_dir" || exit 1

    # Create a build directory if it doesn't exist
    if [ ! -d "build" ]; then
        mkdir build
    fi

    # Navigate to the build directory
    cd build || exit 1

    # Run the build process
    cmake ..
    make
}

# Build the unitree_legged_sdk library
build_library "$unitree_legged_sdk_path"