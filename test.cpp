#include "ShortestPathLib.h"
#include <iostream>
#include <map>
#include <algorithm>
#include <iomanip>

void printVertexNeighbors(const HE_Vertex* vertex) {
    if (!vertex) {
        std::cout << "  Invalid vertex provided." << std::endl;
        return;
    }

    std::cout << "  Neighbors of Vertex " << vertex->id << " in order: ";

    if (!vertex->leaving) {
        std::cout << "(isolated)" << std::endl;
        return;
    }

    HE_HalfEdge* start_he = vertex->leaving;
    HE_HalfEdge* current_he = start_he;

    do {
        if (!current_he || !current_he->twin) {
            std::cout << "(boundary or corrupted)";
            break; 
        }
        HE_Vertex* neighbor = current_he->twin->origin;
        std::cout << neighbor->id << " ";

        current_he = current_he->twin->next; // Dòng duyệt quan trọng

    } while (current_he != start_he && current_he != nullptr);

    std::cout << std::endl;
}

int main() {
    std::cout << "=== Shortest Path Algorithm Test ===" << std::endl;

    Polyhedron poly;
    std::cout << "\nLoading cube.off..." << std::endl;
    if (!poly.loadFromOFF("models/cube.off")) {
        std::cout << "ERROR: Failed to load mesh" << std::endl;
        return 1;
    }

    // Process algorithm steps
    double epsilon = 0.1;
    poly.computeGeometricParameters(epsilon);
    poly.placeSteinerPoints();
    int original_steiner = poly.getSteinerPoints().size();
    poly.mergeSteinerPoints();
    int final_steiner = poly.getSteinerPoints().size();

    // Build graphs  
    poly.buildApproximationGraph();
    int dense_edges = poly.getGraphEdges().size();
    poly.buildPrunedApproximationGraph();
    int sparse_edges = poly.getGraphEdges().size();
    
    // Results summary
    std::cout << "\nResults:" << std::endl;
    std::cout << "  Mesh: " << poly.getVertices().size() << " vertices, " 
              << poly.getFaces().size() << " faces" << std::endl;
    std::cout << "  Steiner points: " << final_steiner << " (from " << original_steiner << ")" << std::endl;
    std::cout << "  Graph edges: " << sparse_edges << " sparse (from " << dense_edges << " dense)" << std::endl;
    
    double reduction = 100.0 * (dense_edges - sparse_edges) / dense_edges;
    std::cout << "  Edge reduction: " << std::fixed << std::setprecision(1) << reduction << "%" << std::endl;

    // Path finding tests
    std::cout << "\nPath finding:" << std::endl;
    
    // Test vertex-to-vertex
    int start_id = 0, end_id = 5;
    auto result1 = poly.findApproximateShortestPath(start_id, end_id, epsilon);
    if (result1.path_found) {
        std::cout << "  V0->V5: cost=" << std::fixed << std::setprecision(3) 
                  << result1.weighted_cost << ", " << result1.path.size() << " points" << std::endl;
    }
    
    // Test arbitrary surface points on CUBE - using barycentric coordinates
    // Face 0: vertices (0,1,2) = V0=(1,1,-1), V1=(1,-1,-1), V2=(-1,-1,-1), weight=1.0
    // Face 2: vertices (4,7,6) = V4=(1,1,1), V7=(-1,1,1), V6=(-1,-1,1), weight=0.8
    
    // Point 1: On Face 0 with barycentric (0.6, 0.3, 0.1)
    Vector3D start_pos = {
        0.6*1.0 + 0.3*1.0 + 0.1*(-1.0),    // x = 0.6 + 0.3 - 0.1 = 0.8
        0.6*1.0 + 0.3*(-1.0) + 0.1*(-1.0), // y = 0.6 - 0.3 - 0.1 = 0.2
        0.6*(-1.0) + 0.3*(-1.0) + 0.1*(-1.0) // z = -0.6 - 0.3 - 0.1 = -1.0
    };
    
    // Point 2: On Face 2 with barycentric (0.2, 0.5, 0.3)  
    Vector3D end_pos = {
        0.2*1.0 + 0.5*(-1.0) + 0.3*(-1.0),  // x = 0.2 - 0.5 - 0.3 = -0.6
        0.2*1.0 + 0.5*1.0 + 0.3*(-1.0),     // y = 0.2 + 0.5 - 0.3 = 0.4
        0.2*1.0 + 0.5*1.0 + 0.3*1.0         // z = 0.2 + 0.5 + 0.3 = 1.0
    };
    auto result2 = poly.findApproximateShortestPath(start_pos, end_pos, epsilon);
    if (result2.path_found) {
        std::cout << "  Surface: cost=" << std::fixed << std::setprecision(3) 
                  << result2.weighted_cost << ", " << result2.path.size() << " points" << std::endl;
    }
    
    std::cout << "\nCompleted successfully." << std::endl;
    return 0;
}