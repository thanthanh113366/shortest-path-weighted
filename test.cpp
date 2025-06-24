#include "ShortestPathLib.h"
#include <iostream>
#include <map>

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
    if (!poly.loadFromOFF("models/icosahedron.off")) {
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
    poly.mergeSteinerPoints(0.05); // threshold = 0.05
    int merged_count = poly.getSteinerPoints().size();
    
    std::cout << "Points before merging: " << original_count << std::endl;
    std::cout << "Points after merging: " << merged_count << std::endl;
    std::cout << "Reduction: " << (original_count - merged_count) << " points (" 
              << (100.0 * (original_count - merged_count) / original_count) << "%)" << std::endl;

    std::cout << "\n--- Shortest Path Algorithm Test ---" << std::endl;
    int start_id = 0;
    int end_id = 6;
    ShortestPathResult result = poly.findApproximateShortestPath(start_id, end_id, epsilon);

    if (result.path_found) {
        std::cout << "Path found!" << std::endl;
        std::cout << "  - Weighted Cost: " << result.weighted_cost << std::endl;
        std::cout << "  - Number of points in path: " << result.path.size() << std::endl;
    } else {
        std::cout << "Path not found (or algorithm not yet implemented)." << std::endl;
    }

    std::cout << "\n--- Test Finished Successfully ---" << std::endl;
    return 0;
}