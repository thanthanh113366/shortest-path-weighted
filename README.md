# Shortest Path on Weighted Polyhedral Surfaces

A C++ library for computing epsilon-approximate shortest paths on polyhedral surfaces using Half-Edge data structure.

## Overview

This project implements a computational geometry library focused on finding approximate shortest paths on 3D polyhedral surfaces. It uses the Half-Edge data structure for efficient representation and traversal of polyhedral meshes.

## Architecture

### Core Data Structures
- **HE_Vertex**: Vertices with 3D coordinates
- **HE_Face**: Triangular faces with weights
- **HE_HalfEdge**: Directed edges forming the mesh topology

### Key Features
- OFF file format loader with face weights
- Half-Edge data structure implementation
- Paper-compliant polyhedron validation
- Support for both closed and open meshes
- Shortest path algorithm (in development)

## Getting Started

### Prerequisites
- C++ compiler with C++17 support
- Make or CMake (optional)

### Building
```bash
# Compile the test application
g++ -o test_app test.cpp ShortestPathLib.cpp -std=c++17

# Run the test
./test_app
```

### Input Format

**Paper Requirements (Aleksandrov et al.):**
- Simple polyhedron (no self-intersections)
- Triangular faces only
- Positive face weights w_i > 0
- Non-degenerate geometry
- **Both closed and open meshes supported**

**Extended OFF Format:**
```
OFF
<num_vertices> <num_faces> <num_edges>
<vertex_coordinates>
<face_definitions_with_weights>
```

**Face Definition:**
```
3 v1 v2 v3 weight
```

**Example:**
```
OFF
8 12 0
1 1 -1
1 -1 -1
...
3 0 1 2 1.0    # Triangle with vertices 0,1,2 and weight 1.0
3 0 2 3 1.2    # Triangle with vertices 0,2,3 and weight 1.2
...
```

**Validation:**
- Triangular faces verification
- Positive weights checking  
- Geometric degeneracy detection
- Minimum angle computation
- Vertex height calculation (h_v)

## Example Output
```
--- Starting Polyhedron Loader Test ---
Successfully loaded and built half-edge structure from cube.off

--- Verification ---
Basic Stats:
  - Loaded Vertices: 8
  - Loaded Faces: 12
  - Loaded Half-Edges: 36
```

## Development Status

### Completed
- Half-Edge data structure
- OFF file loader
- Mesh topology verification
- Neighbor traversal algorithms

### In Progress
- Epsilon-approximate shortest path algorithm
- Path optimization
- Performance benchmarks

## Project Structure
```
├── ShortestPathLib.h      # Header file with data structures
├── ShortestPathLib.cpp    # Implementation
├── test.cpp              # Test application
├── cube.off              # Sample 3D model (cube)
└── README.md             # This file
```

## Contributing

This is a research/educational project. Contributions are welcome!

## License

This project is open source. Please check the license file for more details.

## Applications

Potential applications include:
- Computer Graphics
- Robotics Path Planning
- Computational Geometry
- 3D Navigation Systems 