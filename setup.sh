#!/bin/bash

package_name="go1_mud_test"
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
unitree_legged_sdk_path="$workspace_dir/src/unitree_legged_sdk"

# Function to build libraries using CMake
build_library() {
    local library_dir="$1"
    cd "$library_dir" || exit 1

    if [ ! -d "build" ]; then
        mkdir build
    fi

    cd build || exit 1
    cmake ..
    make
    cd $workspace_dir
}

# Build the unitree_legged_sdk library
build_library "$unitree_legged_sdk_path"

# Initialize rosdep if not already done
if ! command -v rosdep &> /dev/null; then
    echo "Error: rosdep is not installed. Please install and initialize it, then run this script again."
    exit 1
fi

# Run rosdep install
rosdep install --from-paths "$workspace_dir" --ignore-src --rosdistro noetic

# # Build the workspace
# catkin_make

# # Source the workspace
# source devel/setup.bash

echo "Setup complete!"