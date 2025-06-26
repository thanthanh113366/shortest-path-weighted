# RViz2 Visualization for Shortest Path Algorithm

## Overview
Interactive 3D visualization for the epsilon-approximate shortest path algorithm using RViz2. Shows geometric parameters, Steiner points, and polyhedron structure in real-time.

## Quick Start

### Prerequisites
- ROS2 Humble or compatible
- RViz2 package

### Launch Visualization
```bash
# Easy way using script
./run_visualization.sh

# Manual launch
source /opt/ros/humble/setup.bash
colcon build --packages-select shortest_path_viz
source install/setup.bash
ros2 launch shortest_path_viz visualize.launch.py
```

## Visualization Components

### Polyhedron Mesh
- Red spheres: Vertices of the cube mesh
- Gray lines: Edges connecting vertices
- ROS topic: `/polyhedron_mesh`

### Steiner Points 
- Colored spheres: Steiner points placed on each edge
- Color coding by source vertex:
  - Vertex 0: Red
  - Vertex 1: Green  
  - Vertex 2: Blue
  - Vertex 3: Yellow
  - Vertex 4: Magenta
  - Vertex 5: Cyan
  - Vertex 6: Orange
  - Vertex 7: Purple
- ROS topic: `/steiner_points`

### Geometric Parameters
- White text labels showing parameters for each vertex:
  - `h_v`: Distance to boundary
  - `θ_v`: Minimum angle (degrees)  
  - `r_v`: Sphere radius
  - `δ`: Geometric progression factor
- ROS topic: `/geometric_params`

### Approximation Graph (Task 3)
- **Graph Vertices**: Small spheres representing all vertices in the approximation graph
  - White spheres: Original polyhedron vertices
  - Cyan spheres: Steiner points
- **Graph Edges**: Thin lines showing connections in complete subgraphs
  - Color-coded by weight: Blue (light) → Red (heavy)
  - Semi-transparent to avoid visual clutter
  - Limited display (2000 edges max) for performance
- **Graph Statistics**: Yellow text showing:
  - Total vertices and edges count
  - Weight range
  - Performance info
- ROS topic: `/approximation_graph`

## RViz2 Controls

### Navigation
- Left click + drag: Rotate view
- Right click + drag: Pan view
- Scroll wheel: Zoom in/out
- Middle click: Reset view

### Display Control
- Displays panel: Toggle visibility of components
- MarkerArray: Control individual namespaces
- Grid: Reference coordinate system

## Algorithm Visualization

### Tasks Shown
- Task 1.1-1.2: Half-Edge structure and validation
- Task 2.1: Geometric parameters displayed as text
- Task 2.2: Steiner point placement with geometric progression
- Task 2.3: Merged points (overlaps removed)
- Task 3.1-3.2: Approximation graph with complete subgraphs

### Updates
- Data publishes every 1 second
- Live geometric computations
- Interactive exploration

## Technical Details

### ROS2 Node Structure
```
shortest_path_visualizer
├── Publishers:
│   ├── /polyhedron_mesh (MarkerArray)
│   ├── /steiner_points (MarkerArray)
│   └── /geometric_params (MarkerArray)
├── Frame: map
└── Update Rate: 1 Hz
```

### Coordinate System
- Frame ID: `map`
- Units: Meters
- Origin: Cube center at (0,0,0)

## Troubleshooting

### Common Issues
1. RViz2 not opening: Check `echo $DISPLAY` for X11 forwarding
2. No markers visible: Verify topic subscriptions in Displays panel
3. Build errors: Ensure all ROS2 dependencies installed

### Debug Commands
```bash
# Check topics
ros2 topic list | grep -E "(polyhedron|steiner|geometric|approximation)"

# Monitor messages
ros2 topic echo /steiner_points
ros2 topic echo /approximation_graph

# Node info
ros2 node info /shortest_path_visualizer
```

## Performance

### Optimization
- LOD (Level of Detail): Steiner points scaled by importance
- Culling: Hidden markers not rendered
- Batching: MarkerArrays for efficiency

### Stats for extreme_asymmetric.off
- Original Vertices: 6 (red spheres)
- Edges: 18 (gray lines)  
- Steiner Points: ~300 (colored spheres after merging)
- Graph Vertices: 306 (white + cyan spheres)
- Graph Edges: ~22,814 (showing 2000 for performance)
- Text Labels: 6 (geometric parameters) + 1 (graph stats)

## Future Work

### Planned Features
- Animation of algorithm steps
- Interactive point selection
- Path visualization (Task 3-5)
- Performance metrics overlay
- Multi-mesh comparison

### Advanced Visualization
- Heat maps for face weights
- Vector fields for geometric parameters
- Progressive algorithm visualization
- Real-time parameter tuning

## Notes

- Visualization shows cube.off mesh by default
- Epsilon = 0.1 for approximation
- Color scheme optimized for accessibility
- Compatible with ROS2 Humble and newer 