#include "ShortestPathLib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <map>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

    // Handle boundary edges (following paper specification - open mesh allowed)
    if (!edgeMap.empty()) {
        std::cout << "Info: " << edgeMap.size() << " boundary edges found. Open mesh detected." << std::endl;
        // Mark boundary half-edges (twin = nullptr indicates boundary)
        for (const auto& [edgeKey, he] : edgeMap) {
            he->twin = nullptr; // Explicitly mark as boundary edge
        }
    }
    
    // Validate according to paper requirements
    if (!validatePolyhedronRequirements()) {
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

// --- Paper-compliant validation functions ---

bool Polyhedron::validatePolyhedronRequirements() {
    std::cout << "Validating polyhedron according to paper requirements..." << std::endl;
    
    // 1. Check triangular faces only
    for (const auto& face : faces_) {
        if (!isTriangularFace(face.get())) {
            std::cerr << "Error: Face " << face->id << " is not triangular. Paper requires triangular faces only." << std::endl;
            return false;
        }
    }
    
    // 2. Check positive weights
    for (const auto& face : faces_) {
        if (face->weight <= 0.0) {
            std::cerr << "Error: Face " << face->id << " has non-positive weight " << face->weight 
                      << ". Paper requires positive weights." << std::endl;
            return false;
        }
    }
    
    // 3. Check geometric parameters (non-degenerate)
    double min_angle = M_PI; // Start with max possible angle
    for (const auto& face : faces_) {
        double face_min_angle = computeMinimumAngle(face.get());
        if (face_min_angle <= 0.0) {
            std::cerr << "Error: Face " << face->id << " has degenerate geometry (min angle = " 
                      << face_min_angle << "). Paper requires non-degenerate triangles." << std::endl;
            return false;
        }
        min_angle = std::min(min_angle, face_min_angle);
    }
    
    // 4. Check vertex heights (h_v > 0)
    for (const auto& vertex : vertices_) {
        double h_v = computeVertexHeight(vertex.get());
        if (h_v <= 0.0) {
            std::cerr << "Error: Vertex " << vertex->id << " has invalid height h_v = " << h_v 
                      << ". Paper requires positive vertex heights." << std::endl;
            return false;
        }
    }
    
    std::cout << "✓ All paper requirements satisfied:" << std::endl;
    std::cout << "  - Triangular faces: " << faces_.size() << std::endl;
    std::cout << "  - Positive weights: ✓" << std::endl;
    std::cout << "  - Minimum angle: " << (min_angle * 180.0 / M_PI) << "°" << std::endl;
    std::cout << "  - Non-degenerate geometry: ✓" << std::endl;
    
    return true;
}

bool Polyhedron::isTriangularFace(const HE_Face* face) {
    if (!face || !face->edge) return false;
    
    int vertex_count = 0;
    HE_HalfEdge* current = face->edge;
    do {
        vertex_count++;
        current = current->next;
        if (vertex_count > 3) return false; // Early exit for efficiency
    } while (current && current != face->edge);
    
    return vertex_count == 3;
}

double Polyhedron::computeMinimumAngle(const HE_Face* face) {
    if (!face || !face->edge) return 0.0;
    
    // Get the three vertices of the triangle
    HE_HalfEdge* he1 = face->edge;
    HE_HalfEdge* he2 = he1->next;
    HE_HalfEdge* he3 = he2->next;
    
    Vector3D v1 = he1->origin->pos;
    Vector3D v2 = he2->origin->pos;
    Vector3D v3 = he3->origin->pos;
    
    // Compute three angles using dot product
    auto computeAngle = [](const Vector3D& a, const Vector3D& b, const Vector3D& c) {
        Vector3D ba = {a.x - b.x, a.y - b.y, a.z - b.z};
        Vector3D bc = {c.x - b.x, c.y - b.y, c.z - b.z};
        
        double dot = ba.x * bc.x + ba.y * bc.y + ba.z * bc.z;
        double mag_ba = std::sqrt(ba.x * ba.x + ba.y * ba.y + ba.z * ba.z);
        double mag_bc = std::sqrt(bc.x * bc.x + bc.y * bc.y + bc.z * bc.z);
        
        if (mag_ba <= 1e-10 || mag_bc <= 1e-10) return 0.0; // Degenerate
        
        double cos_angle = dot / (mag_ba * mag_bc);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle)); // Clamp to valid range
        return std::acos(cos_angle);
    };
    
    double angle1 = computeAngle(v3, v1, v2);
    double angle2 = computeAngle(v1, v2, v3);
    double angle3 = computeAngle(v2, v3, v1);
    
    return std::min({angle1, angle2, angle3});
}

double Polyhedron::computeVertexHeight(const HE_Vertex* vertex) {
    if (!vertex || !vertex->leaving) return 0.0;
    
    // Simplified computation: minimum distance to any incident edge that doesn't contain this vertex
    double min_height = std::numeric_limits<double>::max();
    
    // Traverse all incident faces
    HE_HalfEdge* start_he = vertex->leaving;
    HE_HalfEdge* current_he = start_he;
    
    do {
        if (current_he->next && current_he->next->next) {
            // Get the opposite edge of the triangle
            HE_HalfEdge* opposite_he = current_he->next;
            Vector3D edge_start = opposite_he->origin->pos;
            Vector3D edge_end = opposite_he->next->origin->pos;
            
            // Compute distance from vertex to this edge
            Vector3D v_pos = vertex->pos;
            Vector3D edge_vec = {edge_end.x - edge_start.x, edge_end.y - edge_start.y, edge_end.z - edge_start.z};
            Vector3D to_vertex = {v_pos.x - edge_start.x, v_pos.y - edge_start.y, v_pos.z - edge_start.z};
            
            double edge_length_sq = edge_vec.x * edge_vec.x + edge_vec.y * edge_vec.y + edge_vec.z * edge_vec.z;
            
            if (edge_length_sq > 1e-10) {
                double t = (to_vertex.x * edge_vec.x + to_vertex.y * edge_vec.y + to_vertex.z * edge_vec.z) / edge_length_sq;
                t = std::max(0.0, std::min(1.0, t)); // Clamp to edge
                
                Vector3D closest = {
                    edge_start.x + t * edge_vec.x,
                    edge_start.y + t * edge_vec.y,
                    edge_start.z + t * edge_vec.z
                };
                
                double dist = distance(v_pos, closest);
                min_height = std::min(min_height, dist);
            }
        }
        
        current_he = current_he->twin ? current_he->twin->next : nullptr;
    } while (current_he && current_he != start_he);
    
    return (min_height == std::numeric_limits<double>::max()) ? 1.0 : min_height; // Default fallback
}

// --- Task 2.1: Geometric Parameters Computation ---

bool Polyhedron::computeGeometricParameters(double epsilon) {
    std::cout << "\n=== Task 2.1: Computing Geometric Parameters ===" << std::endl;
    
    if (epsilon <= 0.0 || epsilon >= 0.5) {
        std::cerr << "Error: Epsilon must be in range (0, 0.5). Got: " << epsilon << std::endl;
        return false;
    }
    
    // Initialize parameters vector
    vertex_params_.clear();
    vertex_params_.resize(vertices_.size());
    
    std::cout << "Computing parameters for " << vertices_.size() << " vertices..." << std::endl;
    
    for (size_t i = 0; i < vertices_.size(); ++i) {
        const HE_Vertex* vertex = vertices_[i].get();
        VertexGeometricParams& params = vertex_params_[i];
        
        // 1. Compute h_v (vertex height)
        params.h_v = computeVertexHeight_v2(vertex);
        
        // 2. Compute θ_v (minimum angle)
        params.theta_v = computeVertexMinAngle(vertex);
        
        // 3. Compute r_v = ε * h_v (sphere radius)
        params.r_v = epsilon * params.h_v;
        
        // 4. Compute δ = 1 + ε * sin(θ_v) (geometric progression parameter)
        params.delta = 1.0 + epsilon * std::sin(params.theta_v);
        
        // Validation
        if (params.h_v <= 0.0 || params.theta_v <= 0.0) {
            std::cerr << "Error: Invalid parameters for vertex " << i 
                      << " (h_v=" << params.h_v << ", θ_v=" << params.theta_v << ")" << std::endl;
            return false;
        }
    }
    
    displayGeometricParameters();
    return true;
}

double Polyhedron::computeVertexHeight_v2(const HE_Vertex* vertex) {
    if (!vertex || !vertex->leaving) return 1.0; // Fallback for isolated vertices
    
    // h_v = minimum distance from vertex to boundary of union of incident faces
    // This is the distance to the nearest edge that is NOT incident to this vertex
    
    double min_distance = std::numeric_limits<double>::max();
    std::set<const HE_HalfEdge*> incident_edges;
    
    // Collect all edges incident to this vertex
    HE_HalfEdge* start_he = vertex->leaving;
    HE_HalfEdge* current_he = start_he;
    
    do {
        incident_edges.insert(current_he);
        if (current_he->twin) {
            incident_edges.insert(current_he->twin);
        }
        current_he = current_he->twin ? current_he->twin->next : nullptr;
    } while (current_he && current_he != start_he);
    
    // Find minimum distance to non-incident edges
    for (const auto& he_ptr : halfEdges_) {
        const HE_HalfEdge* edge = he_ptr.get();
        
        // Skip if this edge is incident to our vertex
        if (incident_edges.count(edge) > 0) continue;
        if (edge->origin == vertex) continue;
        if (edge->next && edge->next->origin == vertex) continue;
        
        // Compute distance from vertex to this edge
        Vector3D edge_start = edge->origin->pos;
        Vector3D edge_end = edge->next ? edge->next->origin->pos : edge_start;
        
        double dist = computePointToLineDistance(vertex->pos, edge_start, edge_end);
        min_distance = std::min(min_distance, dist);
    }
    
    return (min_distance == std::numeric_limits<double>::max()) ? 1.0 : min_distance;
}

double Polyhedron::computeVertexMinAngle(const HE_Vertex* vertex) {
    if (!vertex || !vertex->leaving) return M_PI / 6.0; // Default 30 degrees
    
    // θ_v = minimum angle between any two adjacent edges at this vertex
    double min_angle = M_PI;
    
    HE_HalfEdge* start_he = vertex->leaving;
    HE_HalfEdge* current_he = start_he;
    
    do {
        if (current_he->twin && current_he->twin->next) {
            // Get two adjacent outgoing edges from this vertex
            HE_HalfEdge* edge1 = current_he;
            HE_HalfEdge* edge2 = current_he->twin->next;
            
            Vector3D v_pos = vertex->pos;
            Vector3D p1 = edge1->next ? edge1->next->origin->pos : v_pos;
            Vector3D p2 = edge2->next ? edge2->next->origin->pos : v_pos;
            
            // Compute angle between vectors v_pos->p1 and v_pos->p2
            Vector3D vec1 = {p1.x - v_pos.x, p1.y - v_pos.y, p1.z - v_pos.z};
            Vector3D vec2 = {p2.x - v_pos.x, p2.y - v_pos.y, p2.z - v_pos.z};
            
            double mag1 = std::sqrt(vec1.x*vec1.x + vec1.y*vec1.y + vec1.z*vec1.z);
            double mag2 = std::sqrt(vec2.x*vec2.x + vec2.y*vec2.y + vec2.z*vec2.z);
            
            if (mag1 > 1e-10 && mag2 > 1e-10) {
                double dot = vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z;
                double cos_angle = dot / (mag1 * mag2);
                cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
                
                double angle = std::acos(cos_angle);
                min_angle = std::min(min_angle, angle);
            }
        }
        
        current_he = current_he->twin ? current_he->twin->next : nullptr;
    } while (current_he && current_he != start_he);
    
    return min_angle;
}

void Polyhedron::displayGeometricParameters() const {
    std::cout << "\n--- Geometric Parameters (Task 2.1) ---" << std::endl;
    std::cout << "Vertex | h_v     | θ_v(°)  | r_v     | δ       " << std::endl;
    std::cout << "-------|---------|---------|---------|----------" << std::endl;
    
    for (size_t i = 0; i < vertex_params_.size(); ++i) {
        const VertexGeometricParams& params = vertex_params_[i];
        std::cout << std::setw(6) << i << " | "
                  << std::setw(7) << std::fixed << std::setprecision(3) << params.h_v << " | "
                  << std::setw(7) << std::fixed << std::setprecision(1) << (params.theta_v * 180.0 / M_PI) << " | "
                  << std::setw(7) << std::fixed << std::setprecision(3) << params.r_v << " | "
                  << std::setw(8) << std::fixed << std::setprecision(3) << params.delta
                  << std::endl;
    }
    std::cout << std::endl;
}

const VertexGeometricParams& Polyhedron::getVertexGeometricParams(int vertexID) const {
    assert(vertexID >= 0 && vertexID < vertex_params_.size() && "Vertex ID out of bounds for geometric parameters.");
    return vertex_params_[vertexID];
}

// Helper function for point-to-line distance
double computePointToLineDistance(const Vector3D& point, const Vector3D& line_start, const Vector3D& line_end) {
    Vector3D line_vec = {line_end.x - line_start.x, line_end.y - line_start.y, line_end.z - line_start.z};
    Vector3D point_vec = {point.x - line_start.x, point.y - line_start.y, point.z - line_start.z};
    
    double line_length_sq = line_vec.x*line_vec.x + line_vec.y*line_vec.y + line_vec.z*line_vec.z;
    
    if (line_length_sq < 1e-10) {
        // Degenerate line, return distance to point
        return std::sqrt(point_vec.x*point_vec.x + point_vec.y*point_vec.y + point_vec.z*point_vec.z);
    }
    
    double t = (point_vec.x*line_vec.x + point_vec.y*line_vec.y + point_vec.z*line_vec.z) / line_length_sq;
    t = std::max(0.0, std::min(1.0, t)); // Clamp to line segment
    
    Vector3D closest = {
        line_start.x + t * line_vec.x,
        line_start.y + t * line_vec.y,
        line_start.z + t * line_vec.z
    };
    
    return distance(point, closest);
}

ShortestPathResult Polyhedron::findApproximateShortestPath(int startVertexID, int endVertexID, double epsilon) {
    assert(startVertexID >= 0 && startVertexID < vertices_.size() && "Start vertex ID is out of bounds.");
    assert(endVertexID >= 0 && endVertexID < vertices_.size() && "End vertex ID is out of bounds.");
    assert(epsilon > 0.0 && epsilon < 0.5 && "Epsilon must be in the range (0, 0.5).");
    
    // Compute geometric parameters if not already done
    if (vertex_params_.empty()) {
        std::cout << "Computing geometric parameters first..." << std::endl;
        if (!computeGeometricParameters(epsilon)) {
            ShortestPathResult result;
            return result;
        }
    }
    
    ShortestPathResult result;
    std::cout << "\n--- Shortest Path Algorithm Test ---" << std::endl;
    std::cout << "  - From Vertex: " << startVertexID << std::endl;
    std::cout << "  - To Vertex: " << endVertexID << std::endl;
    std::cout << "  - Epsilon: " << epsilon << std::endl;
    
    std::cout << "Algorithm logic not yet implemented." << std::endl;
    return result; 
}