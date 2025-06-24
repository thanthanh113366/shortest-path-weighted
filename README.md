# Shortest Path on Weighted Polyhedral Surfaces

A C++ library for computing epsilon-approximate shortest paths on polyhedral surfaces using Half-Edge data structure.

## 📋 Overview

This project implements a computational geometry library focused on finding approximate shortest paths on 3D polyhedral surfaces. It uses the Half-Edge data structure for efficient representation and traversal of polyhedral meshes.

## 🏗️ Architecture

### Core Data Structures
- **HE_Vertex**: Vertices with 3D coordinates
- **HE_Face**: Triangular faces with weights
- **HE_HalfEdge**: Directed edges forming the mesh topology

### Key Features
- ✅ OFF file format loader
- ✅ Half-Edge data structure implementation
- ✅ Mesh topology verification
- ⚠️ Shortest path algorithm (in development)

## 🚀 Getting Started

### Prerequisites
- C++ compiler with C++11 support
- Make or CMake (optional)

### Building
```bash
# Compile the test application
g++ -o test_app test.cpp ShortestPathLib.cpp -std=c++11

# Run the test
./test_app
```

### Input Format
The library accepts 3D models in OFF (Object File Format):
```
OFF
<num_vertices> <num_faces> <num_edges>
<vertex_coordinates>
<face_definitions>
```

## 📊 Example Output
```
--- Starting Polyhedron Loader Test ---
Successfully loaded and built half-edge structure from cube.off

--- Verification ---
Basic Stats:
  - Loaded Vertices: 8
  - Loaded Faces: 12
  - Loaded Half-Edges: 36
```

## 🔧 Development Status

### Completed
- [x] Half-Edge data structure
- [x] OFF file loader
- [x] Mesh topology verification
- [x] Neighbor traversal algorithms

### In Progress
- [ ] Epsilon-approximate shortest path algorithm
- [ ] Path optimization
- [ ] Performance benchmarks

## 📁 Project Structure
```
├── ShortestPathLib.h      # Header file with data structures
├── ShortestPathLib.cpp    # Implementation
├── test.cpp              # Test application
├── cube.off              # Sample 3D model (cube)
└── README.md             # This file
```

## 🤝 Contributing

This is a research/educational project. Contributions are welcome!

## 📄 License

This project is open source. Please check the license file for more details.

## 🔬 Applications

Potential applications include:
- Computer Graphics
- Robotics Path Planning
- Computational Geometry
- 3D Navigation Systems 