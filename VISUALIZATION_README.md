# RViz2 Visualization for Shortest Path Algorithm

## Overview
Interactive 3D visualization for the epsilon-approximate shortest path algorithm using RViz2. Shows the complete algorithm implementation including geometric parameters, Steiner points, pruned approximation graph, and two different path finding methods.

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

### Polyhedron Mesh (Task 1)
- **Red spheres**: Vertices of the extreme_asymmetric.off mesh (6 vertices)
- **Gray lines**: Edges connecting vertices
- **Mesh info**: 6 faces with weights ranging from 0.2 to 5.8
- **ROS topic**: `/polyhedron_mesh`

### Steiner Points (Task 2)
- **Colored spheres**: Steiner points placed using geometric progression
- **Before merging**: 303 points → **After merging**: 300 points (3 eliminated)
- **Color coding by source vertex**:
  - Vertex 0: Red
  - Vertex 1: Green  
  - Vertex 2: Blue
  - Vertex 3: Yellow
  - Vertex 4: Magenta
  - Vertex 5: Cyan
- **ROS topic**: `/steiner_points`

### Geometric Parameters (Task 2.1)
- **White text labels** showing parameters for each vertex:
  - `h_v`: Distance to boundary of incident faces
  - `θ_v`: Minimum angle between adjacent edges (degrees)  
  - `r_v`: Sphere radius = ε × h_v
  - `δ`: Geometric progression factor = 1 + ε × sin(θ_v)
- **ROS topic**: `/geometric_params`

### Pruned Approximation Graph G* (Task 5)
- **Graph Vertices**: Small spheres representing all vertices
  - **White spheres**: Original polyhedron vertices (6)
  - **Cyan spheres**: Steiner points (300)
- **Graph Edges**: Thin lines showing pruned connectivity
  - **Color-coded by weight**: Blue (light) → Red (heavy)
  - **Performance optimization**: Dense 22,814 → Sparse 5,987 edges (73.8% reduction)
  - **Display limit**: 2000 edges max for RViz performance
- **Graph Statistics**: Yellow text showing:
  - Vertices: 306 total
  - Steiner reduction: 300 (from 303)
  - Edge reduction: 22,814 → 5,987 (73.8%)
  - Weight range and performance info
- **ROS topic**: `/approximation_graph`

### Shortest Path Results (Task 4)

#### Method 1: Vertex-to-Vertex
- **Magenta path**: V0 → V5 using original vertices
- **Path markers**: Green start, red end, yellow intermediate points
- **Cost**: ~3.119 with 6 path points
- **Info display**: Method, cost, path length, epsilon bound
- **ROS topic**: `/shortest_path`

#### Method 2: Surface Points (Paper-Compliant)
- **Yellow surface path**: (0.5,0.5,0.5) → (-0.3,0.8,-0.2)
- **Orange path markers**: Bright colors for visibility
- **Dynamic vertices**: Temporary vertices added/removed automatically
- **Cost**: ~1.330 with 4 path points
- **Surface projection**: Dense interpolation along polyhedron surface
- **ROS topic**: `/shortest_path_v2`

## RViz2 Controls

### Navigation
- **Left click + drag**: Rotate view
- **Right click + drag**: Pan view  
- **Scroll wheel**: Zoom in/out
- **Middle click**: Reset view

### Display Control
- **Displays panel**: Toggle visibility of components
- **MarkerArray**: Control individual namespaces
- **Grid**: Reference coordinate system

## Algorithm Visualization

### Complete Task Implementation
- **Task 1.1-1.2**: Half-Edge structure and paper-compliant validation ✓
- **Task 2.1**: Geometric parameters (h_v, θ_v, r_v, δ) displayed as text ✓
- **Task 2.2**: Steiner point placement with geometric progression ✓  
- **Task 2.3**: Merged points (overlaps eliminated using interval analysis) ✓
- **Task 3.1-3.2**: Dense approximation graph construction ✓
- **Task 4**: Dijkstra shortest path with two methods ✓
- **Task 5**: Pruned graph optimization (73.8% edge reduction) ✓

### Real-time Updates
- **Data publishing**: Every 1 second
- **Live computations**: Geometric parameters computed dynamically
- **Interactive exploration**: All algorithm components visible simultaneously

## Technical Details

### ROS2 Node Structure
```
shortest_path_visualizer
├── Publishers:
│   ├── /polyhedron_mesh (MarkerArray)
│   ├── /steiner_points (MarkerArray)  
│   ├── /geometric_params (MarkerArray)
│   ├── /approximation_graph (MarkerArray)
│   ├── /shortest_path (MarkerArray)
│   └── /shortest_path_v2 (MarkerArray)
├── Frame: map
└── Update Rate: 1 Hz
```

### Coordinate System
- **Frame ID**: `map`
- **Units**: Meters
- **Origin**: Mesh center at (0,0,0)

## Performance Statistics

### Current Implementation (extreme_asymmetric.off)
- **Original Vertices**: 6 (red spheres)
- **Faces**: 6 with weights [0.2, 5.8, 1.4, 0.6, 3.2, 2.1]
- **Edges**: 18 (gray lines)
- **Steiner Points**: 300 (colored spheres, reduced from 303)
- **Dense Graph**: 22,814 edges → **Sparse Graph**: 5,987 edges
- **Edge Reduction**: 73.8% (Task 5 optimization)
- **Path Method 1**: V0→V5, cost=3.119, 6 points
- **Path Method 2**: Surface points, cost=1.330, 4 points
- **Visualization Load**: ~2,000 edges displayed for performance

### Algorithm Complexity
- **Theoretical**: O(mn log mn) where m ≈ 300 Steiner points
- **Practical Performance**: Graph construction + Dijkstra on sparse graph
- **Memory**: Smart pointers with automatic cleanup

## Troubleshooting

### Common Issues
1. **RViz2 not opening**: Check `echo $DISPLAY` for X11 forwarding
2. **No markers visible**: Verify topic subscriptions in Displays panel
3. **Build errors**: Ensure all ROS2 dependencies installed
4. **Performance lag**: Reduce max_edges_to_show in visualization code

### Debug Commands
```bash
# Check topics
ros2 topic list | grep -E "(polyhedron|steiner|geometric|approximation|shortest)"

# Monitor messages
ros2 topic echo /steiner_points
ros2 topic echo /approximation_graph
ros2 topic echo /shortest_path

# Node info
ros2 node info /shortest_path_visualizer
```

## Future Enhancements

### Planned Features
- **Animation**: Step-by-step algorithm visualization
- **Interactive**: Point selection and parameter tuning
- **Comparison**: Multiple meshes side-by-side
- **Metrics**: Real-time performance analysis

### Advanced Visualization
- **Heat maps**: Face weight visualization
- **Vector fields**: Geometric parameter gradients
- **Progressive**: Algorithm step animation
- **Optimization**: Real-time pruning visualization

## Notes

- **Current mesh**: extreme_asymmetric.off (matches test application)
- **Epsilon**: 0.1 for 10% approximation error bound
- **Color scheme**: Optimized for accessibility and clarity
- **Compatibility**: ROS2 Humble and newer
- **Paper compliance**: Full implementation of Aleksandrov et al. algorithm 