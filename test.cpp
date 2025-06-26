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
    std::cout << "--- Starting Polyhedron Loader Test ---" << std::endl;

    Polyhedron poly;
    if (!poly.loadFromOFF("models/extreme_asymmetric.off")) {
        std::cerr << "Test failed: Could not load or build the polyhedron." << std::endl;
        return 1;
    }

    std::cout << "\n--- Verification ---" << std::endl;
    std::cout << "Basic Stats:" << std::endl;
    std::cout << "  - Loaded Vertices: " << poly.getVertices().size() << std::endl;
    std::cout << "  - Loaded Faces: " << poly.getFaces().size() << std::endl;
    std::cout << "  - Loaded Half-Edges: " << poly.getHalfEdges().size() << std::endl;

    std::cout << "\nTopology Check (Neighbor Traversal):" << std::endl;
    for (const auto& v_ptr : poly.getVertices()) {
        printVertexNeighbors(v_ptr.get());
    }

    std::cout << "\n--- Task 2.1: Geometric Parameters ---" << std::endl;
    double epsilon = 0.1;
    poly.computeGeometricParameters(epsilon);
    poly.displayGeometricParameters();

    std::cout << "\n--- Task 2.2: Steiner Point Placement ---" << std::endl;
    poly.placeSteinerPoints();
    auto& steiner_points = poly.getSteinerPoints();
    std::cout << "Total Steiner points placed: " << steiner_points.size() << std::endl;
    
    // Show distribution by source vertex
    std::map<int, int> vertex_count;
    for (const auto& sp : steiner_points) {
        vertex_count[sp.source_vertex_id]++;
    }
    
    for (const auto& pair : vertex_count) {
        std::cout << "  - From vertex " << pair.first << ": " << pair.second << " points" << std::endl;
    }

    std::cout << "\n--- Task 2.3: Steiner Point Merging ---" << std::endl;
    int original_count = steiner_points.size();
    poly.mergeSteinerPoints(); // Paper algorithm: interval-based elimination
    int merged_count = poly.getSteinerPoints().size();
    
    std::cout << "Points before merging: " << original_count << std::endl;
    std::cout << "Points after merging: " << merged_count << std::endl;
    std::cout << "Reduction: " << (original_count - merged_count) << " points (" 
              << (100.0 * (original_count - merged_count) / original_count) << "%)" << std::endl;

    std::cout << "\n--- Task 3.1 & 3.2: Approximation Graph Construction ---" << std::endl;
    if (poly.buildApproximationGraph()) {
        std::cout << "Approximation graph built successfully!" << std::endl;
        
        // Show detailed statistics
        const auto& graph_vertices = poly.getGraphVertices();
        const auto& graph_edges = poly.getGraphEdges();
        
        std::cout << "\nDetailed breakdown:" << std::endl;
        std::cout << "  - Original vertices in graph: " 
                  << std::count_if(graph_vertices.begin(), graph_vertices.end(),
                                   [](const GraphVertex& gv) { return gv.is_original_vertex; })
                  << std::endl;
        std::cout << "  - Steiner points in graph: " 
                  << std::count_if(graph_vertices.begin(), graph_vertices.end(),
                                   [](const GraphVertex& gv) { return !gv.is_original_vertex; })
                  << std::endl;
        
        if (!graph_edges.empty()) {
            std::cout << "  - Sample edge weights: ";
            for (int i = 0; i < std::min(5, (int)graph_edges.size()); ++i) {
                std::cout << std::fixed << std::setprecision(3) << graph_edges[i].weight << " ";
            }
            std::cout << "..." << std::endl;
        }
    } else {
        std::cout << "Failed to build approximation graph." << std::endl;
    }

    std::cout << "\n--- Task 5: Pruned Graph (G*) Construction Test ---" << std::endl;
    
    // Test Task 5: Build pruned approximation graph
    if (poly.buildPrunedApproximationGraph()) {
        std::cout << "Pruned approximation graph G* built successfully!" << std::endl;
        
        const auto& pruned_vertices = poly.getGraphVertices();
        const auto& pruned_edges = poly.getGraphEdges();
        
        std::cout << "\nPruned graph statistics:" << std::endl;
        std::cout << "  - Vertices in G*: " << pruned_vertices.size() << std::endl;
        std::cout << "  - Edges in G*: " << pruned_edges.size() << std::endl;
    } else {
        std::cout << "Failed to build pruned approximation graph." << std::endl;
    }

    std::cout << "\n--- Shortest Path Algorithm Comparison Test ---" << std::endl;
    
    // Test both versions: vertex-to-vertex and arbitrary surface points
    
    // Version 1: Original vertex-to-vertex with standard Dijkstra
    std::cout << "\n=== Version 1: Standard Dijkstra (Vertex-to-Vertex) ===" << std::endl;
    int start_id = 0;
    int end_id = 5;
    ShortestPathResult result1 = poly.findApproximateShortestPath(start_id, end_id, epsilon);
    
    // Version 2: Paper-compliant arbitrary surface points
    std::cout << "\n=== Version 2: Paper-Compliant Surface Points ===" << std::endl;
    // Let's use vertices from opposite sides to force multi-face path
    // V0 = (0, 1, 1.618) and V2 = (0, 0.618, -1.618) - opposite poles!
    Vector3D source_point = {0.000000, 1.000000, 1.618000};  // Near V0
    Vector3D target_point = {0.000000, 0.618000, -1.618000}; // Near V2
    ShortestPathResult result2 = poly.findApproximateShortestPath(source_point, target_point, epsilon);
    
    // Report results
    ShortestPathResult result = result2.path_found ? result2 : result1; // Prefer surface point result

    if (result.path_found) {
        std::cout << "Path found!" << std::endl;
        std::cout << "  - Weighted Cost: " << result.weighted_cost << std::endl;
        std::cout << "  - Number of points in path: " << result.path.size() << std::endl;
    } else {
        std::cout << "Path not found (Dijkstra algorithm pending - Task 4)." << std::endl;
    }
    
    std::cout << "\n--- Performance Comparison ---" << std::endl;
    std::cout << "Testing both dense (G) and sparse (G*) graphs:" << std::endl;
    
    // Build dense graph for comparison
    std::cout << "\nBuilding dense graph G for comparison..." << std::endl;
    poly.buildApproximationGraph(); // Dense version
    
    // Test dense graph performance
    std::cout << "\nTesting dense graph performance:" << std::endl;
    auto dense_result = poly.findApproximateShortestPath(start_id, end_id, epsilon);
    
    // Build sparse graph
    std::cout << "\nBuilding sparse graph G* for comparison..." << std::endl;
    poly.buildPrunedApproximationGraph(); // Sparse version
    
    // Test sparse graph performance
    std::cout << "\nTesting sparse graph performance:" << std::endl;
    auto sparse_result = poly.findApproximateShortestPath(start_id, end_id, epsilon);
    
    // Compare results
    std::cout << "\n--- Performance Summary ---" << std::endl;
    if (dense_result.path_found && sparse_result.path_found) {
        std::cout << "Dense graph (G)  - Cost: " << std::fixed << std::setprecision(6) 
                  << dense_result.weighted_cost << ", Path length: " << dense_result.path.size() << std::endl;
        std::cout << "Sparse graph (G*) - Cost: " << std::fixed << std::setprecision(6) 
                  << sparse_result.weighted_cost << ", Path length: " << sparse_result.path.size() << std::endl;
        
        double cost_ratio = sparse_result.weighted_cost / dense_result.weighted_cost;
        std::cout << "Cost ratio (G*/G): " << std::fixed << std::setprecision(3) << cost_ratio << std::endl;
        
        if (cost_ratio <= 1.1) { // Within 10% is excellent for sparse approximation
            std::cout << "✓ Sparse graph maintains good approximation quality!" << std::endl;
        } else {
            std::cout << "⚠ Sparse graph has higher cost (expected due to pruning)" << std::endl;
        }
    }

    std::cout << "\n--- Test Finished Successfully ---" << std::endl;
    return 0;
}