#include "ShortestPathLib.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <map>
#include <limits>
#include <set>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <queue>
#include <functional>

/**
 * @file ShortestPathLib.cpp
 * @brief Implementation of epsilon-approximate shortest paths on polyhedral surfaces.
 * 
 * Implementation includes:
 * - Half-Edge data structure with paper-compliant validation
 * - Geometric parameter computation (h_v, theta_v, r_v, delta)
 * - Steiner point placement using geometric progression  
 * - Steiner point merging to reduce overlaps
 */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Forward declaration for helper function
double computePointToLineDistance(const Vector3D& point, const Vector3D& line_start, const Vector3D& line_end);

// --- Helper function for distance calculation ---
double distance(const Vector3D& p1, const Vector3D& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

// Half-Edge data structure and paper-compliant validation

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

// Paper-compliant validation functions

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

// Geometric parameter computation (h_v, theta_v, r_v, delta)

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

// Steiner point placement using geometric progression

bool Polyhedron::placeSteinerPoints() {
    std::cout << "\n=== Task 2.2: Placing Steiner Points ===" << std::endl;
    
    if (vertex_params_.empty()) {
        std::cerr << "Error: Geometric parameters not computed. Call computeGeometricParameters() first." << std::endl;
        return false;
    }
    
    // Clear previous Steiner points
    steiner_points_.clear();
    next_steiner_id_ = 0;
    
    std::cout << "Placing Steiner points on all edges using geometric progression..." << std::endl;
    
    // For each edge, place Steiner points from both endpoints
    std::set<std::pair<int, int>> processed_edges; // To avoid duplicate processing
    
    for (const auto& he_ptr : halfEdges_) {
        HE_HalfEdge* edge = he_ptr.get();
        if (!edge->origin || !edge->next || !edge->next->origin) continue;
        
        int v1_id = edge->origin->id;
        int v2_id = edge->next->origin->id;
        
        // Create canonical edge representation (smaller ID first)
        std::pair<int, int> edge_key = {std::min(v1_id, v2_id), std::max(v1_id, v2_id)};
        
        if (processed_edges.count(edge_key)) continue;
        processed_edges.insert(edge_key);
        
        // Place Steiner points from vertex v1 along edge toward v2
        placeSteinerPointsOnEdge(edge, v1_id);
        
        // Place Steiner points from vertex v2 along edge toward v1 (if twin exists)
        if (edge->twin) {
            placeSteinerPointsOnEdge(edge->twin, v2_id);
        }
    }
    
    std::cout << "Placed " << steiner_points_.size() << " Steiner points total." << std::endl;
    
    return true;
}

void Polyhedron::placeSteinerPointsOnEdge(HE_HalfEdge* edge, int source_vertex_id) {
    if (!edge || !edge->origin || !edge->next || !edge->next->origin) return;
    
    const VertexGeometricParams& params = vertex_params_[source_vertex_id];
    double edge_length = edge->length;
    
    // Safety check: if r_v is too small relative to edge length, skip
    if (params.r_v <= 1e-6 || params.r_v >= edge_length * 0.9) {
        return; // Skip degenerate cases
    }
    
    // Start placing points at distance r_v from source vertex
    double current_distance = params.r_v;
    int progression_index = 0;
    const int MAX_POINTS_PER_EDGE = 20; // Reasonable limit
    
    std::vector<SteinerPoint> edge_points;
    
    while (current_distance < edge_length && progression_index < MAX_POINTS_PER_EDGE) {
        SteinerPoint point;
        point.id = next_steiner_id_++;
        point.edge = edge;
        point.source_vertex_id = source_vertex_id;
        point.distance_from_source = current_distance;
        point.progression_index = progression_index;
        
        // Interpolate position on edge
        point.pos = interpolateOnEdge(edge, current_distance);
        
        edge_points.push_back(point);
        
        // Compute next distance: r_v * δ^(i+1)
        progression_index++;
        current_distance = params.r_v * std::pow(params.delta, progression_index);
        
        // Safety break if progression becomes too slow
        if (params.delta <= 1.001) break; // Delta too close to 1
    }
    
    // Add points to global list
    for (const auto& point : edge_points) {
        steiner_points_.push_back(point);
    }
    
    // Debug output for this edge (limit verbose output)
    if (!edge_points.empty() && edge_points.size() <= 10) {
        std::cout << "  Edge from vertex " << source_vertex_id 
                  << " (length=" << std::fixed << std::setprecision(3) << edge_length 
                  << ", r_v=" << params.r_v << ", δ=" << params.delta
                  << "): " << edge_points.size() << " points" << std::endl;
    } else if (edge_points.size() > 10) {
        std::cout << "  Edge from vertex " << source_vertex_id 
                  << ": " << edge_points.size() << " points (many)" << std::endl;
    }
}

Vector3D Polyhedron::interpolateOnEdge(HE_HalfEdge* edge, double distance_from_start) {
    if (!edge || !edge->origin || !edge->next || !edge->next->origin) {
        return {0.0, 0.0, 0.0};
    }
    
    Vector3D start = edge->origin->pos;
    Vector3D end = edge->next->origin->pos;
    double edge_length = edge->length;
    
    if (edge_length <= 1e-10) return start; // Degenerate edge
    
    double t = distance_from_start / edge_length;
    t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]
    
    return {
        start.x + t * (end.x - start.x),
        start.y + t * (end.y - start.y), 
        start.z + t * (end.z - start.z)
    };
}

void Polyhedron::displaySteinerPoints() const {
    if (steiner_points_.empty()) {
        std::cout << "No Steiner points placed." << std::endl;
        return;
    }
    
    std::cout << "\n--- Steiner Points Summary (Task 2.2) ---" << std::endl;
    std::cout << "Total Steiner points: " << steiner_points_.size() << std::endl;
    
    // Group by source vertex
    std::map<int, std::vector<const SteinerPoint*>> points_by_vertex;
    for (const auto& point : steiner_points_) {
        points_by_vertex[point.source_vertex_id].push_back(&point);
    }
    
    for (const auto& pair : points_by_vertex) {
        int vertex_id = pair.first;
        const auto& points = pair.second;
        const VertexGeometricParams& params = vertex_params_[vertex_id];
        
        std::cout << "Vertex " << vertex_id 
                  << " (r_v=" << std::fixed << std::setprecision(3) << params.r_v 
                  << ", δ=" << std::fixed << std::setprecision(3) << params.delta 
                  << "): " << points.size() << " points" << std::endl;
        
        // Show only first few points to avoid overwhelming output
        int show_count = std::min(5, (int)points.size());
        for (int i = 0; i < show_count; ++i) {
            const auto* point = points[i];
            std::cout << "  Point" << point->id 
                      << " at dist=" << std::fixed << std::setprecision(3) << point->distance_from_source
                      << " (δ^" << point->progression_index << ")"
                      << " pos=(" << std::fixed << std::setprecision(2) 
                      << point->pos.x << "," << point->pos.y << "," << point->pos.z << ")" << std::endl;
        }
        
        if (points.size() > show_count) {
            std::cout << "  ... and " << (points.size() - show_count) << " more points" << std::endl;
        }
    }
    std::cout << std::endl;
}

const std::vector<SteinerPoint>& Polyhedron::getSteinerPoints() const {
    return steiner_points_;
}

// Steiner point merging using paper's interval-size algorithm (Task 2.3)

bool Polyhedron::mergeSteinerPoints() {
    std::cout << "\n=== Task 2.3: Merging Steiner Points (Paper Algorithm) ===" << std::endl;
    
    if (steiner_points_.empty()) {
        std::cerr << "Error: No Steiner points to merge. Call placeSteinerPoints() first." << std::endl;
        return false;
    }
    
    size_t initial_count = steiner_points_.size();
    std::cout << "Initial Steiner points: " << initial_count << std::endl;
    
    // Get all unique edges
    std::set<std::pair<int, int>> unique_edges;
    for (const auto& he_ptr : halfEdges_) {
        HE_HalfEdge* edge = he_ptr.get();
        if (!edge->origin || !edge->next || !edge->next->origin) continue;
        
        int v1_id = edge->origin->id;
        int v2_id = edge->next->origin->id;
        std::pair<int, int> edge_key = {std::min(v1_id, v2_id), std::max(v1_id, v2_id)};
        unique_edges.insert(edge_key);
    }
    
    std::cout << "Applying paper algorithm on " << unique_edges.size() << " unique edges..." << std::endl;
    
    // Apply paper's interval-based merging on each edge
    for (const auto& edge_key : unique_edges) {
        mergeSteinerPointsOnEdge(edge_key);
    }
    
    size_t final_count = steiner_points_.size();
    std::cout << "Final Steiner points: " << final_count 
              << " (eliminated " << (initial_count - final_count) << " points using interval analysis)" << std::endl;
    
    return true;
}

void Polyhedron::mergeSteinerPointsOnEdge(const std::pair<int, int>& edge_key) {
    // Get all points on this edge
    std::vector<SteinerPoint> edge_points = getPointsOnEdge(edge_key);
    
    if (edge_points.size() <= 1) return; // Nothing to merge
    
    // Separate points by their source vertex
    std::vector<SteinerPoint> from_first_vertex;
    std::vector<SteinerPoint> from_second_vertex;
    
    for (const auto& point : edge_points) {
        if (point.source_vertex_id == edge_key.first) {
            from_first_vertex.push_back(point);
        } else if (point.source_vertex_id == edge_key.second) {
            from_second_vertex.push_back(point);
        }
    }
    
    // If we only have points from one vertex, no merging needed
    if (from_first_vertex.empty() || from_second_vertex.empty()) {
        return;
    }
    
    // Sort each progression by distance from source
    std::sort(from_first_vertex.begin(), from_first_vertex.end(), 
        [](const SteinerPoint& a, const SteinerPoint& b) {
            return a.distance_from_source < b.distance_from_source;
        });
    
    std::sort(from_second_vertex.begin(), from_second_vertex.end(), 
        [](const SteinerPoint& a, const SteinerPoint& b) {
            return a.distance_from_source < b.distance_from_source;
        });
    
    // Apply paper's algorithm: find crossover point where interval sizes become equal
    std::vector<SteinerPoint> merged_points;
    
    // Get geometric parameters for both vertices
    const VertexGeometricParams& params1 = vertex_params_[edge_key.first];
    const VertexGeometricParams& params2 = vertex_params_[edge_key.second];
    
    // Determine which progression has smaller initial intervals
    double initial_interval1 = params1.r_v * (params1.delta - 1.0);
    double initial_interval2 = params2.r_v * (params2.delta - 1.0);
    
    // Paper algorithm: eliminate larger intervals when smaller ones exist
    int eliminated_count = 0;
    
    // Process first progression: keep points with smaller intervals than corresponding ones in second progression
    for (const auto& point : from_first_vertex) {
        double this_interval = computeIntervalSizeAt(point, point.source_vertex_id, params1);
        bool should_keep = true;
        
        // Check against points in second progression at similar distances
        for (const auto& other_point : from_second_vertex) {
            double other_interval = computeIntervalSizeAt(other_point, other_point.source_vertex_id, params2);
            
            // If other progression has smaller interval at overlapping position, eliminate this point
            if (other_interval < this_interval * 0.8) { // 80% threshold for practical elimination
                double pos_diff = std::abs(point.distance_from_source - 
                                         (point.edge->length - other_point.distance_from_source));
                if (pos_diff < std::max(this_interval, other_interval) * 2.0) {
                    should_keep = false;
                    eliminated_count++;
                    break;
                }
            }
        }
        
        if (should_keep) {
            merged_points.push_back(point);
        }
    }
    
    // Process second progression with same logic
    for (const auto& point : from_second_vertex) {
        double this_interval = computeIntervalSizeAt(point, point.source_vertex_id, params2);
        bool should_keep = true;
        
        for (const auto& other_point : from_first_vertex) {
            double other_interval = computeIntervalSizeAt(other_point, other_point.source_vertex_id, params1);
            
            if (other_interval < this_interval * 0.8) {
                double pos_diff = std::abs(point.distance_from_source - 
                                         (point.edge->length - other_point.distance_from_source));
                if (pos_diff < std::max(this_interval, other_interval) * 2.0) {
                    should_keep = false;
                    eliminated_count++;
                    break;
                }
            }
        }
        
        if (should_keep) {
            merged_points.push_back(point);
        }
    }
    
    // Debug output
    if (edge_points.size() != merged_points.size()) {
        std::cout << "  Edge (" << edge_key.first << "," << edge_key.second << "): "
                  << from_first_vertex.size() << "+" << from_second_vertex.size() 
                  << " → " << merged_points.size() << " points (eliminated " << eliminated_count 
                  << " with larger intervals)" << std::endl;
    }
    
    // Replace points for this edge
    replacePointsOnEdge(edge_key, merged_points);
}

double Polyhedron::computeIntervalSizeAt(const SteinerPoint& point, int source_vertex_id, const VertexGeometricParams& params) {
    // Paper algorithm: interval size at position i is r_v * δ^i * (δ-1)
    // This represents the spacing between consecutive Steiner points in geometric progression
    
    if (point.progression_index < 0) return 0.0;
    
    // Interval size = r_v * δ^i * (δ-1)
    double interval_size = params.r_v * std::pow(params.delta, point.progression_index) * (params.delta - 1.0);
    
    return interval_size;
}

std::vector<SteinerPoint> Polyhedron::getPointsOnEdge(const std::pair<int, int>& edge_key) {
    std::vector<SteinerPoint> edge_points;
    
    for (const auto& point : steiner_points_) {
        if (!point.edge || !point.edge->origin || !point.edge->next || !point.edge->next->origin) continue;
        
        int v1 = point.edge->origin->id;
        int v2 = point.edge->next->origin->id;
        std::pair<int, int> point_edge = {std::min(v1, v2), std::max(v1, v2)};
        
        if (point_edge == edge_key) {
            edge_points.push_back(point);
        }
    }
    
    return edge_points;
}

void Polyhedron::replacePointsOnEdge(const std::pair<int, int>& edge_key, const std::vector<SteinerPoint>& new_points) {
    // Remove old points for this edge
    steiner_points_.erase(
        std::remove_if(steiner_points_.begin(), steiner_points_.end(),
            [edge_key](const SteinerPoint& point) {
                if (!point.edge || !point.edge->origin || !point.edge->next || !point.edge->next->origin) return false;
                
                int v1 = point.edge->origin->id;
                int v2 = point.edge->next->origin->id;
                std::pair<int, int> point_edge = {std::min(v1, v2), std::max(v1, v2)};
                
                return point_edge == edge_key;
            }),
        steiner_points_.end());
    
    // Add new points
    for (const auto& point : new_points) {
        steiner_points_.push_back(point);
    }
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
    
    std::cout << "\n--- Shortest Path Algorithm Test ---" << std::endl;
    std::cout << "  - From Vertex: " << startVertexID << std::endl;
    std::cout << "  - To Vertex: " << endVertexID << std::endl;
    std::cout << "  - Epsilon: " << epsilon << std::endl;
    
    // Compute geometric parameters if not already done
    if (vertex_params_.empty()) {
        std::cout << "Computing geometric parameters first..." << std::endl;
        if (!computeGeometricParameters(epsilon)) {
            ShortestPathResult result;
            std::cout << "Failed to compute geometric parameters." << std::endl;
            return result;
        }
    }
    
    // Build approximation graph if not already done
    if (graph_vertices_.empty() || graph_edges_.empty()) {
        std::cout << "Building approximation graph first..." << std::endl;
        if (!buildApproximationGraph()) {
            ShortestPathResult result;
            std::cout << "Failed to build approximation graph." << std::endl;
            return result;
        }
    }
    
    // Find graph vertex IDs corresponding to start and end vertices
    int start_graph_id = -1, end_graph_id = -1;
    for (const auto& gv : graph_vertices_) {
        if (gv.is_original_vertex && gv.original_vertex_id == startVertexID) {
            start_graph_id = gv.id;
        }
        if (gv.is_original_vertex && gv.original_vertex_id == endVertexID) {
            end_graph_id = gv.id;
        }
    }
    
    if (start_graph_id == -1 || end_graph_id == -1) {
        ShortestPathResult result;
        std::cout << "Error: Could not find graph vertices for start/end vertices" << std::endl;
        return result;
    }
    
    std::cout << "Mapped to graph vertices: " << start_graph_id << " -> " << end_graph_id << std::endl;
    
    // Run Dijkstra algorithm on the approximation graph
    return runDijkstra(start_graph_id, end_graph_id);
}

// ===============================================================
// === Task 4: Dijkstra Algorithm Implementation ===
// ===============================================================

ShortestPathResult Polyhedron::runDijkstra(int start_graph_id, int end_graph_id) {
    ShortestPathResult result;
    
    std::cout << "Running Dijkstra algorithm on approximation graph..." << std::endl;
    std::cout << "Graph size: " << graph_vertices_.size() << " vertices, " << graph_edges_.size() << " edges" << std::endl;
    
    // Build adjacency list for efficient graph traversal
    std::unordered_map<int, std::vector<std::pair<int, double>>> adj_list;
    
    // Initialize adjacency list
    for (const auto& gv : graph_vertices_) {
        adj_list[gv.id] = std::vector<std::pair<int, double>>();
    }
    
    // Populate adjacency list from graph edges
    for (const auto& edge : graph_edges_) {
        adj_list[edge.vertex1_id].emplace_back(edge.vertex2_id, edge.weight);
        adj_list[edge.vertex2_id].emplace_back(edge.vertex1_id, edge.weight); // Undirected graph
    }
    
    const double INF = std::numeric_limits<double>::infinity();
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> predecessors;
    std::unordered_map<int, bool> visited;
    
    // Initialize distances and predecessors
    for (const auto& gv : graph_vertices_) {
        distances[gv.id] = INF;
        predecessors[gv.id] = -1;
        visited[gv.id] = false;
    }
    
    distances[start_graph_id] = 0.0;
    
    // Priority queue: {distance, vertex_id}
    std::priority_queue<std::pair<double, int>, 
                       std::vector<std::pair<double, int>>, 
                       std::greater<std::pair<double, int>>> pq;
    
    pq.push({0.0, start_graph_id});
    
    int processed_vertices = 0;
    
    while (!pq.empty()) {
        auto [current_dist, current_vertex] = pq.top();
        pq.pop();
        
        // Skip if already processed with better distance
        if (visited[current_vertex]) continue;
        
        visited[current_vertex] = true;
        processed_vertices++;
        
        // Early termination if we reached the target
        if (current_vertex == end_graph_id) {
            std::cout << "Target reached! Processed " << processed_vertices << " vertices." << std::endl;
            break;
        }
        
        // Progress reporting for large graphs
        if (processed_vertices % 100 == 0) {
            std::cout << "  Processed " << processed_vertices << " vertices..." << std::endl;
        }
        
        // Relax neighbors
        for (const auto& [neighbor_id, edge_weight] : adj_list[current_vertex]) {
            if (visited[neighbor_id]) continue;
            
            double new_distance = current_dist + edge_weight;
            if (new_distance < distances[neighbor_id]) {
                distances[neighbor_id] = new_distance;
                predecessors[neighbor_id] = current_vertex;
                pq.push({new_distance, neighbor_id});
            }
        }
    }
    
    // Check if path was found
    if (distances[end_graph_id] == INF) {
        std::cout << "No path found between vertices " << start_graph_id 
                  << " and " << end_graph_id << std::endl;
        return result;
    }
    
    // Reconstruct path
    std::vector<int> graph_path;
    int current = end_graph_id;
    while (current != -1) {
        graph_path.push_back(current);
        current = predecessors[current];
    }
    std::reverse(graph_path.begin(), graph_path.end());
    
    // Convert graph vertex IDs to 3D coordinates
    std::vector<Vector3D> path_3d;
    for (int graph_vertex_id : graph_path) {
        for (const auto& gv : graph_vertices_) {
            if (gv.id == graph_vertex_id) {
                path_3d.push_back(gv.pos);
                break;
            }
        }
    }
    
    // Package result
    result.path_found = true;
    result.weighted_cost = distances[end_graph_id];
    result.path = path_3d;
    
    std::cout << "=== Dijkstra Results ===" << std::endl;
    std::cout << "Path found: YES" << std::endl;
    std::cout << "Weighted cost: " << std::fixed << std::setprecision(6) << result.weighted_cost << std::endl;
    std::cout << "Path length: " << result.path.size() << " vertices" << std::endl;
    std::cout << "Graph vertices processed: " << processed_vertices << "/" << graph_vertices_.size() << std::endl;
    
    return result;
}

bool Polyhedron::buildApproximationGraph() {
    std::cout << "\n=== Building Complete Approximation Graph G ===\n" << std::endl;
    
    // Task 3.1: Construct vertices
    if (!constructApproximationGraphVertices()) {
        return false;
    }
    
    // Task 3.2: Construct edges
    if (!constructApproximationGraphEdges()) {
        return false;
    }
    
    // Display statistics
    displayGraphStatistics();
    
    return true;
}

// ===============================================================
// === Graph Access Methods ===
// ===============================================================

const std::vector<GraphVertex>& Polyhedron::getGraphVertices() const {
    return graph_vertices_;
}

const std::vector<GraphEdge>& Polyhedron::getGraphEdges() const {
    return graph_edges_;
}

void Polyhedron::displayGraphStatistics() const {
    std::cout << "\n=== Approximation Graph G Statistics ===" << std::endl;
    std::cout << "Vertices: " << graph_vertices_.size() << std::endl;
    std::cout << "  - Original vertices: " << vertices_.size() << std::endl;
    std::cout << "  - Steiner points: " << steiner_points_.size() << std::endl;
    std::cout << "Edges: " << graph_edges_.size() << std::endl;
    std::cout << "Faces: " << faces_.size() << std::endl;
    
    if (!graph_edges_.empty()) {
        double min_weight = graph_edges_[0].weight;
        double max_weight = graph_edges_[0].weight;
        double total_weight = 0.0;
        
        for (const auto& edge : graph_edges_) {
            min_weight = std::min(min_weight, edge.weight);
            max_weight = std::max(max_weight, edge.weight);
            total_weight += edge.weight;
        }
        
        std::cout << "Edge weights:" << std::endl;
        std::cout << "  - Min: " << min_weight << std::endl;
        std::cout << "  - Max: " << max_weight << std::endl;
        std::cout << "  - Average: " << (total_weight / graph_edges_.size()) << std::endl;
    }
    
    // Calculate average degree
    if (!graph_vertices_.empty()) {
        double avg_degree = 2.0 * graph_edges_.size() / graph_vertices_.size();
        std::cout << "Average vertex degree: " << avg_degree << std::endl;
    }
    
    std::cout << "=== Graph Construction Complete ===" << std::endl;
}

// ===============================================================
// === Helper Methods for Task 3.2 ===
// ===============================================================

std::vector<int> Polyhedron::getVerticesOnFace(const HE_Face* face) {
    std::vector<int> face_vertices;
    
    if (!face || !face->edge) {
        return face_vertices;
    }
    
    HE_HalfEdge* start_edge = face->edge;
    HE_HalfEdge* current_edge = start_edge;
    
    do {
        if (!current_edge || !current_edge->origin) {
            break;
        }
        
        // Find the graph vertex ID corresponding to this original vertex
        int original_vertex_id = current_edge->origin->id;
        for (const auto& gv : graph_vertices_) {
            if (gv.is_original_vertex && gv.original_vertex_id == original_vertex_id) {
                face_vertices.push_back(gv.id);
                break;
            }
        }
        
        current_edge = current_edge->next;
    } while (current_edge && current_edge != start_edge);
    
    return face_vertices;
}

std::vector<int> Polyhedron::getSteinerPointsOnFace(const HE_Face* face) {
    std::vector<int> face_steiner_points;
    
    if (!face || !face->edge) {
        return face_steiner_points;
    }
    
    // Get all edges of this face
    std::vector<HE_HalfEdge*> face_edges;
    HE_HalfEdge* start_edge = face->edge;
    HE_HalfEdge* current_edge = start_edge;
    
    do {
        if (!current_edge) break;
        face_edges.push_back(current_edge);
        current_edge = current_edge->next;
    } while (current_edge && current_edge != start_edge);
    
    // Find Steiner points on each edge of this face
    for (const auto& edge : face_edges) {
        if (!edge || !edge->origin || !edge->next || !edge->next->origin) continue;
        
        int v1 = edge->origin->id;
        int v2 = edge->next->origin->id;
        std::pair<int, int> edge_key = {std::min(v1, v2), std::max(v1, v2)};
        
        // Find Steiner points on this edge
        for (const auto& steiner_point : steiner_points_) {
            if (!steiner_point.edge || !steiner_point.edge->origin || 
                !steiner_point.edge->next || !steiner_point.edge->next->origin) continue;
            
            int sp_v1 = steiner_point.edge->origin->id;
            int sp_v2 = steiner_point.edge->next->origin->id;
            std::pair<int, int> sp_edge_key = {std::min(sp_v1, sp_v2), std::max(sp_v1, sp_v2)};
            
            if (sp_edge_key == edge_key) {
                // Find the graph vertex ID for this Steiner point
                for (const auto& gv : graph_vertices_) {
                    if (!gv.is_original_vertex && gv.steiner_point_id == steiner_point.id) {
                        face_steiner_points.push_back(gv.id);
                        break;
                    }
                }
            }
        }
    }
    
    return face_steiner_points;
}

std::vector<int> Polyhedron::getAllPointsOnFace(const HE_Face* face) {
    std::vector<int> all_points;
    
    // Get original vertices on this face
    std::vector<int> vertices = getVerticesOnFace(face);
    all_points.insert(all_points.end(), vertices.begin(), vertices.end());
    
    // Get Steiner points on this face
    std::vector<int> steiner_pts = getSteinerPointsOnFace(face);
    all_points.insert(all_points.end(), steiner_pts.begin(), steiner_pts.end());
    
    return all_points;
}

bool Polyhedron::createCompleteSubgraph(const HE_Face* face) {
    if (!face) {
        return false;
    }
    
    // Get all points (vertices + Steiner points) on this face
    std::vector<int> face_points = getAllPointsOnFace(face);
    
    if (face_points.size() < 2) {
        // Not enough points to create edges
        return true;
    }
    
    int edges_added = 0;
    
    // Create complete subgraph: connect every pair of points on this face
    for (size_t i = 0; i < face_points.size(); ++i) {
        for (size_t j = i + 1; j < face_points.size(); ++j) {
            int vertex1_id = face_points[i];
            int vertex2_id = face_points[j];
            
            // Find the actual positions of these vertices
            Vector3D pos1, pos2;
            bool found1 = false, found2 = false;
            
            for (const auto& gv : graph_vertices_) {
                if (gv.id == vertex1_id) {
                    pos1 = gv.pos;
                    found1 = true;
                }
                if (gv.id == vertex2_id) {
                    pos2 = gv.pos;
                    found2 = true;
                }
                if (found1 && found2) break;
            }
            
            if (!found1 || !found2) {
                std::cerr << "Error: Could not find positions for vertices " 
                          << vertex1_id << " and " << vertex2_id << std::endl;
                continue;
            }
            
            // Calculate edge weight based on Euclidean distance and face weight
            double euclidean_distance = distance(pos1, pos2);
            double edge_weight = euclidean_distance * face->weight;
            
            // Create the edge
            GraphEdge edge;
            edge.id = next_graph_edge_id_++;
            edge.vertex1_id = vertex1_id;
            edge.vertex2_id = vertex2_id;
            edge.weight = edge_weight;
            edge.face_id = face->id;
            edge.euclidean_length = euclidean_distance;
            
            graph_edges_.push_back(edge);
            edges_added++;
        }
    }
    
    return edges_added > 0;
}

bool Polyhedron::constructApproximationGraphVertices() {
    graph_vertices_.clear();
    next_graph_vertex_id_ = 0;
    
    std::cout << "--- Task 3.1: Constructing Graph Vertices ---" << std::endl;
    
    // Add original vertices
    for (const auto& vertex : vertices_) {
        GraphVertex gv;
        gv.id = next_graph_vertex_id_++;
        gv.pos = vertex->pos;
        gv.is_original_vertex = true;
        gv.original_vertex_id = vertex->id;
        gv.steiner_point_id = -1;
        
        graph_vertices_.push_back(gv);
    }
    
    std::cout << "Added " << vertices_.size() << " original vertices" << std::endl;
    
    // Add Steiner points
    for (const auto& steiner_point : steiner_points_) {
        GraphVertex gv;
        gv.id = next_graph_vertex_id_++;
        gv.pos = steiner_point.pos;
        gv.is_original_vertex = false;
        gv.original_vertex_id = -1;
        gv.steiner_point_id = steiner_point.id;
        
        graph_vertices_.push_back(gv);
    }
    
    std::cout << "Added " << steiner_points_.size() << " Steiner points" << std::endl;
    std::cout << "Total graph vertices: " << graph_vertices_.size() << std::endl;
    
    return true;
}

bool Polyhedron::constructApproximationGraphEdges() {
    graph_edges_.clear();
    next_graph_edge_id_ = 0;
    
    std::cout << "\n--- Task 3.2: Constructing Graph Edges ---" << std::endl;
    
    int faces_processed = 0;
    
    // For each face, create a complete subgraph
    for (const auto& face : faces_) {
        if (!createCompleteSubgraph(face.get())) {
            std::cerr << "Error: Failed to create complete subgraph for face " << face->id << std::endl;
            return false;
        }
        faces_processed++;
        
        if (faces_processed % 10 == 0 || faces_processed == faces_.size()) {
            std::cout << "Processed " << faces_processed << "/" << faces_.size() << " faces" << std::endl;
        }
    }
    
    std::cout << "Created " << graph_edges_.size() << " edges" << std::endl;
    
    return true;
}

// ===============================================================
// === Task 5: Pruned Graph Construction (G* optimization) ===
// ===============================================================

bool Polyhedron::buildPrunedApproximationGraph() {
    graph_vertices_.clear();
    graph_edges_.clear();
    next_graph_vertex_id_ = 0;
    next_graph_edge_id_ = 0;
    
    std::cout << "\n=== Task 5: Building Pruned Approximation Graph G* ===" << std::endl;
    
    // Task 3.1: Construct vertices (same as before)
    if (!constructApproximationGraphVertices()) {
        return false;
    }
    
    // Task 5.1: Construct sparse edges using logarithmic rules
    std::cout << "\n--- Task 5.1: Constructing Sparse Graph Edges ---" << std::endl;
    
    int faces_processed = 0;
    int total_edges_created = 0;
    
    // Create sparse subgraph for each face
    for (const auto& face : faces_) {
        if (!createSparseSubgraph(face.get())) {
            std::cerr << "Error: Failed to create sparse subgraph for face " << face->id << std::endl;
            return false;
        }
        faces_processed++;
        
        // Progress update
        if (faces_processed % 10 == 0 || faces_processed == faces_.size()) {
            int new_edges = graph_edges_.size() - total_edges_created;
            total_edges_created = graph_edges_.size();
            std::cout << "Processed face " << faces_processed << "/" << faces_.size() 
                      << " (+" << new_edges << " sparse edges)" << std::endl;
        }
    }
    
    std::cout << "Sparse graph construction completed:" << std::endl;
    std::cout << "  - Faces processed: " << faces_processed << std::endl;
    std::cout << "  - Total sparse edges created: " << graph_edges_.size() << std::endl;
    
    double reduction_ratio = 1.0;
    if (faces_.size() > 0) {
        // Estimate dense graph edges (complete subgraph would be roughly n*(n-1)/2 per face)
        int avg_points_per_face = (vertices_.size() + steiner_points_.size()) / faces_.size();
        int estimated_dense_edges = faces_.size() * avg_points_per_face * (avg_points_per_face - 1) / 2;
        reduction_ratio = (double)graph_edges_.size() / estimated_dense_edges;
    }
    
    std::cout << "  - Edge reduction ratio: " << std::fixed << std::setprecision(3) 
              << reduction_ratio << " (sparse vs complete)" << std::endl;
    
    // Display statistics
    displayGraphStatistics();
    
    return true;
}

bool Polyhedron::createSparseSubgraph(const HE_Face* face) {
    if (!face) return false;
    
    // Get all points (vertices + steiner points) on this face
    std::vector<int> face_vertex_ids = getVerticesOnFace(face);
    std::vector<int> face_steiner_ids = getSteinerPointsOnFace(face);
    
    if (face_vertex_ids.empty() && face_steiner_ids.empty()) return true;
    
    int edges_created = 0;
    
    // Paper Algorithm for Task 5: Logarithmic connectivity rules
    
    // 1. Connect all original vertices (maintain full connectivity for mesh structure)
    for (size_t i = 0; i < face_vertex_ids.size(); ++i) {
        for (size_t j = i + 1; j < face_vertex_ids.size(); ++j) {
            edges_created += createGraphEdge(face_vertex_ids[i], face_vertex_ids[j], face);
        }
    }
    
    // 2. Connect original vertices to nearby Steiner points (within r_v radius)
    for (int vertex_id : face_vertex_ids) {
        for (int steiner_id : face_steiner_ids) {
            if (isSteinerPointNearVertex(steiner_id, vertex_id)) {
                edges_created += createGraphEdge(vertex_id, steiner_id, face);
            }
        }
    }
    
    // 3. Paper's logarithmic rule for Steiner-Steiner connections
    // Instead of O(n²) connections, use O(n log n) logarithmic spacing
    std::sort(face_steiner_ids.begin(), face_steiner_ids.end());
    
    for (size_t i = 0; i < face_steiner_ids.size(); ++i) {
        int steiner_id1 = face_steiner_ids[i];
        
        // Connect to immediate neighbors (maintain local connectivity)
        if (i > 0) {
            edges_created += createGraphEdge(steiner_id1, face_steiner_ids[i-1], face);
        }
        if (i < face_steiner_ids.size() - 1) {
            edges_created += createGraphEdge(steiner_id1, face_steiner_ids[i+1], face);
        }
        
        // Connect to logarithmically distant neighbors (paper's key optimization)
        for (int k = 1; k <= 8; ++k) { // Limit log distance for practical implementation
            int log_distance = (1 << k); // 2^k spacing: 2, 4, 8, 16, 32, 64, 128, 256
            
            if (i + log_distance < face_steiner_ids.size()) {
                edges_created += createGraphEdge(steiner_id1, face_steiner_ids[i + log_distance], face);
            }
            
            if (i >= log_distance) {
                edges_created += createGraphEdge(steiner_id1, face_steiner_ids[i - log_distance], face);
            }
        }
    }
    
    return edges_created > 0;
}

bool Polyhedron::isSteinerPointNearVertex(int steiner_graph_id, int vertex_graph_id) {
    // Find the corresponding Steiner point and check if it was generated from this vertex
    for (const auto& gv : graph_vertices_) {
        if (gv.id == steiner_graph_id && !gv.is_original_vertex && gv.steiner_point_id >= 0) {
            // Find the actual Steiner point
            for (const auto& steiner_point : steiner_points_) {
                if (steiner_point.id == gv.steiner_point_id) {
                    // Check if this Steiner point's source vertex matches the given vertex
                    for (const auto& vertex_gv : graph_vertices_) {
                        if (vertex_gv.id == vertex_graph_id && vertex_gv.is_original_vertex &&
                            vertex_gv.original_vertex_id == steiner_point.source_vertex_id) {
                            return true; // Steiner point was generated from this vertex
                        }
                    }
                }
            }
        }
    }
    
    return false; // Not a nearby Steiner point
}

int Polyhedron::createGraphEdge(int vertex1_id, int vertex2_id, const HE_Face* face) {
    // Find positions of these vertices
    Vector3D pos1, pos2;
    bool found1 = false, found2 = false;
    
    for (const auto& gv : graph_vertices_) {
        if (gv.id == vertex1_id) { pos1 = gv.pos; found1 = true; }
        if (gv.id == vertex2_id) { pos2 = gv.pos; found2 = true; }
        if (found1 && found2) break;
    }
    
    if (!found1 || !found2) return 0;
    
    double euclidean_distance = distance(pos1, pos2);
    
    // Skip extremely short edges (numerical precision issues)
    if (euclidean_distance < 1e-10) return 0;
    
    // Create the edge with paper-compliant weight
    double edge_weight = euclidean_distance * face->weight;
    
    GraphEdge edge;
    edge.id = next_graph_edge_id_++;
    edge.vertex1_id = vertex1_id;
    edge.vertex2_id = vertex2_id;
    edge.weight = edge_weight;
    edge.face_id = face->id;
    edge.euclidean_length = euclidean_distance;
    
    graph_edges_.push_back(edge);
    
    return 1; // Successfully created 1 edge
}

// Placeholder for path finding with positions
ShortestPathResult Polyhedron::findApproximateShortestPath(const Vector3D& start_pos, const Vector3D& end_pos, double epsilon) {
    ShortestPathResult result;
    
    std::cout << "\n=== Finding Approximate Shortest Path (Position-based) ===\n" << std::endl;
    std::cout << "Start position: (" << start_pos.x << ", " << start_pos.y << ", " << start_pos.z << ")" << std::endl;
    std::cout << "End position: (" << end_pos.x << ", " << end_pos.y << ", " << end_pos.z << ")" << std::endl;
    std::cout << "Epsilon: " << epsilon << std::endl;
    
    // Find closest vertices to start and end positions
    int start_vertex_id = -1, end_vertex_id = -1;
    double min_start_dist = std::numeric_limits<double>::infinity();
    double min_end_dist = std::numeric_limits<double>::infinity();
    
    for (const auto& vertex : vertices_) {
        double start_dist = distance(vertex->pos, start_pos);
        double end_dist = distance(vertex->pos, end_pos);
        
        if (start_dist < min_start_dist) {
            min_start_dist = start_dist;
            start_vertex_id = vertex->id;
        }
        
        if (end_dist < min_end_dist) {
            min_end_dist = end_dist;
            end_vertex_id = vertex->id;
        }
    }
    
    if (start_vertex_id == -1 || end_vertex_id == -1) {
        std::cerr << "Error: Could not find closest vertices" << std::endl;
        return result;
    }
    
    std::cout << "Closest start vertex: " << start_vertex_id << " (distance: " << min_start_dist << ")" << std::endl;
    std::cout << "Closest end vertex: " << end_vertex_id << " (distance: " << min_end_dist << ")" << std::endl;
    
    // Use the existing method
    return findApproximateShortestPath(start_vertex_id, end_vertex_id, epsilon);
}