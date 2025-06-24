#!/bin/bash

echo "Shortest Path Weighted Polyhedron Visualization"
echo "=============================================="

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 not sourced! Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Found ROS2 $ROS_DISTRO"

# Build the workspace if needed
echo "Building workspace..."
colcon build --packages-select shortest_path_viz

if [ $? -eq 0 ]; then
    echo "Build successful"
else
    echo "Build failed"
    exit 1
fi

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "Launching RViz2..."
echo ""
echo "Visualization components:"
echo "- Red spheres: Cube vertices"
echo "- Gray lines: Polyhedron edges"
echo "- Colored spheres: Steiner points (each color = different source vertex)"
echo "- Text labels: Geometric parameters for each vertex"
echo ""
echo "Controls: Mouse to rotate/zoom, Displays panel to toggle visibility"
echo ""

# Launch the visualization
ros2 launch shortest_path_viz visualize.launch.py 