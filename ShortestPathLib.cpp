#include "ShortestPathLib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <map>

// --- Helper function for distance calculation ---
double distance(const Vector3D& p1, const Vector3D& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// --- Polyhedron Class Implementation ---

Polyhedron::~Polyhedron() {
    // std::unique_ptr will automatically handle memory deallocation.
}

bool Polyhedron::loadFromOFF(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return false;
    }

    std::string line;
    std::getline(file, line);
    if (line.find("OFF") == std::string::npos) {
        std::cerr << "Error: Invalid OFF file format. Missing 'OFF' keyword." << std::endl;
        return false;
    }

    int numVertices, numFaces, numEdges;
    file >> numVertices >> numFaces >> numEdges;
    std::getline(file, line); 

    std::vector<Vector3D> temp_vertices(numVertices);
    for (int i = 0; i < numVertices; ++i) {
        file >> temp_vertices[i].x >> temp_vertices[i].y >> temp_vertices[i].z;
    }

    std::vector<std::vector<int>> temp_faces_from_file;
    std::vector<double> temp_face_weights;
    for (int i = 0; i < numFaces; ++i) {
        int n_verts_in_face;
        file >> n_verts_in_face;
        if (n_verts_in_face > 0) {
            std::vector<int> face_indices(n_verts_in_face);
            for (int j = 0; j < n_verts_in_face; ++j) {
                file >> face_indices[j];
            }
            temp_faces_from_file.push_back(face_indices);
            
            // Read weight for this face
            double weight = 1.0; // Default weight
            file >> weight;
            temp_face_weights.push_back(weight);
        }
    }
    file.close();

    // Clear any old data
    vertices_.clear();
    faces_.clear();
    halfEdges_.clear();
    
    vertices_.reserve(numVertices);
    for (int i = 0; i < numVertices; ++i) {
        auto v = std::make_unique<HE_Vertex>();
        v->id = i;
        v->pos = temp_vertices[i];
        vertices_.push_back(std::move(v));
    }

    using EdgeKey = std::pair<int, int>;
    std::map<EdgeKey, HE_HalfEdge*> edgeMap;

    for (size_t face_idx = 0; face_idx < temp_faces_from_file.size(); ++face_idx) {
        const auto& face_indices = temp_faces_from_file[face_idx];
        auto f = std::make_unique<HE_Face>();
        f->id = faces_.size();
        f->weight = temp_face_weights[face_idx]; // Set weight from file
        
        HE_Face* current_face = f.get();
        faces_.push_back(std::move(f)); 

        int num_face_verts = face_indices.size();
        std::vector<HE_HalfEdge*> face_half_edges(num_face_verts);

        for (int i = 0; i < num_face_verts; ++i) {
            auto he = std::make_unique<HE_HalfEdge>();
            he->id = halfEdges_.size();
            he->face = current_face;
            he->origin = vertices_[face_indices[i]].get();
            face_half_edges[i] = he.get();
            halfEdges_.push_back(std::move(he));
        }
        
        current_face->edge = face_half_edges[0];

        for (int i = 0; i < num_face_verts; ++i) {
            int next_i = (i + 1) % num_face_verts;
            face_half_edges[i]->next = face_half_edges[next_i];

            int v_start_idx = face_indices[i];
            int v_end_idx = face_indices[next_i];
            
            face_half_edges[i]->length = distance(vertices_[v_start_idx]->pos, vertices_[v_end_idx]->pos);

            EdgeKey key = {std::min(v_start_idx, v_end_idx), std::max(v_start_idx, v_end_idx)};

            if (edgeMap.count(key)) {
                face_half_edges[i]->twin = edgeMap[key];
                edgeMap[key]->twin = face_half_edges[i];
                edgeMap.erase(key);
            } else {
                edgeMap[key] = face_half_edges[i];
            }
        }
    }

    for (const auto& he_ptr : halfEdges_) {
        if (he_ptr->origin) {
            he_ptr->origin->leaving = he_ptr.get();
        }
    }

    if (!edgeMap.empty()) {
        std::cerr << "Error: " << edgeMap.size() << " boundary edges found. Mesh must be watertight." << std::endl;
        return false;
    }
    
    std::cout << "Successfully loaded and built half-edge structure from " << filename << std::endl;
    std::cout << "Face weights loaded: ";
    for (size_t i = 0; i < temp_face_weights.size(); ++i) {
        std::cout << "F" << i << "=" << temp_face_weights[i] << " ";
        if ((i + 1) % 6 == 0) std::cout << std::endl << "                     "; // Line break every 6 faces
    }
    std::cout << std::endl;
    return true;
}

ShortestPathResult Polyhedron::findApproximateShortestPath(int startVertexID, int endVertexID, double epsilon) {
    assert(startVertexID >= 0 && startVertexID < vertices_.size() && "Start vertex ID is out of bounds.");
    assert(endVertexID >= 0 && endVertexID < vertices_.size() && "End vertex ID is out of bounds.");
    assert(epsilon > 0.0 && epsilon < 0.5 && "Epsilon must be in the range (0, 0.5).");
    
    ShortestPathResult result;
    std::cout << "Starting shortest path calculation..." << std::endl;
    std::cout << "  - From Vertex: " << startVertexID << std::endl;
    std::cout << "  - To Vertex: " << endVertexID << std::endl;
    std::cout << "  - Epsilon: " << epsilon << std::endl;
    
    std::cout << "Algorithm logic not yet implemented." << std::endl;
    return result; 
}