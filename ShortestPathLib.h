#ifndef SHORTEST_PATH_LIB_H
#define SHORTEST_PATH_LIB_H

#include <vector>
#include <string>
#include <map>
#include <utility> // For std::pair
#include <memory>  // For std::unique_ptr
#include <unordered_map>

/**
 * @file ShortestPathLib.h
 * @brief Library for computing epsilon-approximate shortest paths on polyhedral surfaces.
 * 
 * Based on "Approximating Shortest Paths on a Polyhedral Surface" by Aleksandrov et al.
 * 
 * Current implementation:
 * - Half-Edge data structure and OFF file loading
 * - Paper-compliant validation (triangular faces, positive weights, non-degenerate geometry)
 * - Geometric parameter computation (h_v, theta_v, r_v, delta) for all vertices  
 * - Steiner point placement on edges using geometric progression
 * - Steiner point merging to remove overlapping points on each edge
 * 
 * TODO:
 * - Approximation graph construction
 * - Dijkstra shortest path algorithm on approximation graph
 * - Graph pruning optimization
 */

// --- Core Half-Edge Data Structures ---

// Forward declarations
struct HE_Vertex;
struct HE_Face;
struct HE_HalfEdge;

struct Vector3D {
    double x = 0.0, y = 0.0, z = 0.0;
};

/**
 * @struct VertexGeometricParams
 * @brief Geometric parameters for each vertex as defined in the paper (Task 2.1)
 */
struct VertexGeometricParams {
    double h_v = 0.0;     // Minimum distance from vertex to boundary of union of incident faces
    double theta_v = 0.0; // Minimum angle between any two adjacent edges at this vertex  
    double r_v = 0.0;     // Sphere radius = epsilon * h_v (computed when epsilon is known)
    double delta = 0.0;   // Geometric progression parameter = 1 + epsilon * sin(theta_v)
};

/**
 * @struct SteinerPoint
 * @brief Represents a Steiner point placed on an edge (Task 2.2)
 */
struct SteinerPoint {
    int id;                    // Unique ID for this Steiner point
    Vector3D pos;              // 3D position on the edge
    HE_HalfEdge* edge;         // The edge this point lies on
    int source_vertex_id;      // Vertex from which this point was generated
    double distance_from_source; // Distance from source vertex along the edge
    int progression_index;     // Index in geometric progression (0=r_v, 1=r_v*δ, 2=r_v*δ², ...)
};

/**
 * @struct GraphVertex
 * @brief Represents a vertex in the approximation graph G (Task 3.1)
 */
struct GraphVertex {
    int id;                     // Unique ID for this graph vertex
    Vector3D pos;              // 3D position
    bool is_original_vertex;   // true if this is an original polyhedron vertex
    int original_vertex_id;    // ID of original vertex (if is_original_vertex = true)
    int steiner_point_id;      // ID of Steiner point (if is_original_vertex = false)
};

/**
 * @struct GraphEdge
 * @brief Represents an edge in the approximation graph G (Task 3.2)
 */
struct GraphEdge {
    int id;                    // Unique ID for this graph edge
    int vertex1_id;            // First vertex ID in the graph
    int vertex2_id;            // Second vertex ID in the graph
    double weight;             // Edge weight = euclidean_distance × face_weight
    int face_id;               // ID of the face this edge lies across
    double euclidean_length;   // Pure Euclidean distance
};

/**
 * @struct ShortestPathResult
 * @brief Contains the result of the shortest path finding algorithm.
 */
struct ShortestPathResult {
    bool path_found = false;    // Flag indicating whether a path was found successfully
    double weighted_cost = 0.0; // Total (weighted) cost of the path
    std::vector<Vector3D> path; // List of 3D points forming the path
};

/**
 * @struct HE_Vertex
 * @brief Represents a vertex in the Half-Edge data structure.
 */
struct HE_Vertex {
    int id;
    Vector3D pos;
    HE_HalfEdge* leaving = nullptr; // One of the half-edges originating from this vertex.
};

/**
 * @struct HE_Face
 * @brief Represents a triangular face in the Half-Edge data structure.
 */
struct HE_Face {
    int id;
    double weight = 1.0;
    HE_HalfEdge* edge = nullptr; // One of the half-edges on the boundary of this face.
};

/**
 * @struct HE_HalfEdge
 * @brief Represents a directed edge on the boundary of a face. The core of the data structure.
 */
struct HE_HalfEdge {
    int id;
    HE_Vertex* origin = nullptr;     // Vertex this half-edge starts from.
    HE_HalfEdge* twin = nullptr;     // The opposite half-edge in the adjacent face.
    HE_HalfEdge* next = nullptr;     // The next half-edge in a CCW loop around the face.
    HE_Face* face = nullptr;         // The face this half-edge belongs to.
    double length = 0.0;             // Length of the full edge.
};

/**
 * @class Polyhedron
 * @brief Container class for the entire polyhedron using Half-Edge structure.
 * 
 * Implements the epsilon-approximation algorithm for weighted shortest paths
 * on polyhedral surfaces according to Aleksandrov et al. paper.
 * 
 * Features:
 * - Half-Edge topology for efficient surface navigation
 * - Paper-compliant validation (no watertight requirement)
 * - Geometric parameter computation for approximation quality
 * - Steiner point placement and merging for surface discretization
 * - Memory management using smart pointers
 */
class Polyhedron {
public:
    Polyhedron() : next_steiner_id_(0), next_graph_vertex_id_(0), next_graph_edge_id_(0) {}
    ~Polyhedron();

    // Disable copy semantics
    Polyhedron(const Polyhedron&) = delete;
    Polyhedron& operator=(const Polyhedron&) = delete;

    // === Task 1.1 & 1.2: Data Structure and Loading ===
    
    /**
     * @brief Loads polyhedron from OFF file with paper-compliant validation (Tasks 1.1 & 1.2)
     * @param filename Path to OFF file with face weights
     * @return true if loading and validation successful
     */
    bool loadFromOFF(const std::string& filename);

    // Basic accessors
    const std::vector<std::unique_ptr<HE_Vertex>>& getVertices() const { return vertices_; }
    const std::vector<std::unique_ptr<HE_Face>>& getFaces() const { return faces_; }
    const std::vector<std::unique_ptr<HE_HalfEdge>>& getHalfEdges() const { return halfEdges_; }

    // === Task 2.1: Geometric Parameters ===
    
    /**
     * @brief Computes geometric parameters (h_v, θ_v, r_v, δ) for all vertices (Task 2.1)
     * @param epsilon Approximation factor used to compute r_v and delta parameters
     * @return true if all parameters computed successfully
     */
    bool computeGeometricParameters(double epsilon);
    
    /**
     * @brief Get geometric parameters for a specific vertex
     * @param vertexID ID of the vertex
     * @return const reference to geometric parameters
     */
    const VertexGeometricParams& getVertexGeometricParams(int vertexID) const;
    
    // === Task 2.2: Steiner Point Placement ===
    
    /**
     * @brief Places Steiner points on all edges using geometric progression (Task 2.2)
     * Uses formula: r_v, r_v×δ, r_v×δ², r_v×δ³, ... from both endpoints
     * @return true if Steiner points placed successfully
     */
    bool placeSteinerPoints();
    
    // === Task 2.3: Steiner Point Merging ===
    
    /**
     * @brief Merges overlapping Steiner points on each edge (Task 2.3)
     * Paper algorithm: Removes points where interval sizes from two geometric progressions become equal,
     * eliminating larger intervals when smaller intervals overlap
     * @return true if merging completed successfully
     */
    bool mergeSteinerPoints();
    
    /**
     * @brief Get all Steiner points after placement and merging
     * @return const reference to vector of Steiner points
     */
    const std::vector<SteinerPoint>& getSteinerPoints() const;
    
    /**
     * @brief Display geometric parameters for all vertices (Task 2.1)
     */
    void displayGeometricParameters() const;
    
    // === Task 3.1: Graph Vertex Construction ===
    
    /**
     * @brief Constructs vertices for the approximation graph G (Task 3.1)
     * Combines original polyhedron vertices and Steiner points
     * @return true if graph vertices constructed successfully
     */
    bool constructApproximationGraphVertices();
    
    // === Task 3.2: Graph Edge Construction ===
    
    /**
     * @brief Constructs edges for the approximation graph G (Task 3.2)
     * Creates complete subgraph for each face with weighted edges
     * @return true if graph edges constructed successfully
     */
    bool constructApproximationGraphEdges();
    
    /**
     * @brief Builds the complete approximation graph G (Tasks 3.1 + 3.2)
     * @return true if graph construction successful
     */
    bool buildApproximationGraph();
    
    // === Task 5: Pruned Graph Construction (Optional Optimization) ===
    
    /**
     * @brief Constructs pruned approximation graph G* (Task 5.1)
     * Creates sparse graph using logarithmic connectivity rules
     * @return true if pruned graph constructed successfully
     */
    bool buildPrunedApproximationGraph();
    
    /**
     * @brief Creates sparse subgraph for a face using logarithmic rules (Task 5.1)
     * @param face Face to create sparse connectivity for
     * @return true if sparse subgraph created successfully
     */
    bool createSparseSubgraph(const HE_Face* face);
    
    // === Graph Access Methods ===
    
    /**
     * @brief Get all vertices in the approximation graph
     * @return const reference to vector of graph vertices
     */
    const std::vector<GraphVertex>& getGraphVertices() const;
    
    /**
     * @brief Get all edges in the approximation graph
     * @return const reference to vector of graph edges
     */
    const std::vector<GraphEdge>& getGraphEdges() const;
    
    /**
     * @brief Display approximation graph statistics
     */
    void displayGraphStatistics() const;
    
    /**
     * @brief Computes the epsilon-approximate shortest path on the surface of a polyhedron.
     * @param startVertexID ID of the start vertex.
     * @param endVertexID ID of the end vertex.
     * @param epsilon Approximation factor (e.g., 0.1 for 10% error). Must be > 0.
     * @return a ShortestPathResult struct containing the path and its cost.
     */
    ShortestPathResult findApproximateShortestPath(
        int startVertexID, 
        int endVertexID, 
        double epsilon
    );

    /**
     * @brief Paper-compliant version: shortest path between arbitrary surface points
     * @param source_point 3D coordinates of source point on polyhedron surface
     * @param target_point 3D coordinates of target point on polyhedron surface  
     * @param epsilon Approximation factor (e.g., 0.1 for 10% error). Must be > 0.
     * @return a ShortestPathResult struct containing the path and its cost.
     */
    ShortestPathResult findApproximateShortestPath(
        const Vector3D& source_point,
        const Vector3D& target_point, 
        double epsilon
    );

private:
    // === Task 4: Dijkstra Algorithm ===
    
    /**
     * @brief Runs Dijkstra algorithm on the approximation graph (Task 4)
     * @param start_graph_id Graph vertex ID of start point
     * @param end_graph_id Graph vertex ID of end point  
     * @return ShortestPathResult with path and cost
     */
    ShortestPathResult runDijkstra(int start_graph_id, int end_graph_id);
    
    /**
     * @brief Optimized Dijkstra with Fibonacci heap (Paper-compliant O(mn log mn))
     * @param start_graph_id Graph vertex ID of start point
     * @param end_graph_id Graph vertex ID of end point  
     * @return ShortestPathResult with path and cost
     */
    ShortestPathResult runOptimizedDijkstra(int start_graph_id, int end_graph_id);
    
    /**
     * @brief Reconstructs path from Dijkstra predecessor array
     * @param predecessors Array of predecessor graph vertex IDs
     * @param start_graph_id Start graph vertex ID
     * @param end_graph_id End graph vertex ID
     * @return Vector of 3D points forming the path
     */
    std::vector<Vector3D> reconstructPath(const std::vector<int>& predecessors, 
                                          int start_graph_id, 
                                          int end_graph_id);
    
    // === Paper-compliant Surface Point Methods ===
    
    /**
     * @brief Find face containing a given 3D point on polyhedron surface
     * @param point 3D coordinates of point
     * @return Pointer to face containing the point, or nullptr if not found
     */
    const HE_Face* findFaceContainingPoint(const Vector3D& point);
    
    /**
     * @brief Check if point lies inside triangular face using barycentric coordinates
     * @param point 3D coordinates of point
     * @param face Triangular face to test
     * @return true if point is inside face
     */
    bool isPointInTriangle(const Vector3D& point, const HE_Face* face);
    
    /**
     * @brief Add temporary vertex to approximation graph for surface point
     * @param surface_point 3D coordinates of point on surface
     * @param containing_face Face that contains this point
     * @return Graph vertex ID of the added temporary vertex
     */
    int addTemporaryGraphVertex(const Vector3D& surface_point, const HE_Face* containing_face);
    
    /**
     * @brief Connect temporary vertex to nearby points on the same face
     * @param temp_vertex_id Graph vertex ID of temporary vertex
     * @param containing_face Face containing the temporary vertex
     */
    void connectTemporaryVertex(int temp_vertex_id, const HE_Face* containing_face);
    
    /**
     * @brief Remove temporary vertices added for a specific shortest path computation
     * @param temp_vertex_ids List of temporary vertex IDs to remove
     */
    void removeTemporaryVertices(const std::vector<int>& temp_vertex_ids);

private:
    // === Core Data Structures ===
    std::vector<std::unique_ptr<HE_Vertex>> vertices_;
    std::vector<std::unique_ptr<HE_Face>> faces_;
    std::vector<std::unique_ptr<HE_HalfEdge>> halfEdges_;
    std::vector<VertexGeometricParams> vertex_params_;  // Task 2.1: Geometric parameters per vertex
    std::vector<SteinerPoint> steiner_points_;         // Task 2.2-2.3: Steiner points on edges
    int next_steiner_id_;                              // Counter for unique Steiner point IDs
    
    // === Task 3: Approximation Graph G ===
    std::vector<GraphVertex> graph_vertices_;          // Task 3.1: All vertices in approximation graph
    std::vector<GraphEdge> graph_edges_;               // Task 3.2: All edges in approximation graph
    int next_graph_vertex_id_;                         // Counter for unique graph vertex IDs
    int next_graph_edge_id_;                           // Counter for unique graph edge IDs
    
    // === Task 1.2: Paper-Compliant Validation ===
    bool validatePolyhedronRequirements();
    bool isTriangularFace(const HE_Face* face);
    double computeMinimumAngle(const HE_Face* face);
    double computeVertexHeight(const HE_Vertex* vertex);
    
    // === Task 2.1: Geometric Parameter Computation ===
    double computeVertexHeight_v2(const HE_Vertex* vertex);      // h_v: distance to boundary
    double computeVertexMinAngle(const HE_Vertex* vertex);       // θ_v: minimum angle
    
    // === Task 2.2: Steiner Point Placement ===
    void placeSteinerPointsOnEdge(HE_HalfEdge* edge, int source_vertex_id);
    Vector3D interpolateOnEdge(HE_HalfEdge* edge, double distance_from_start);
    void displaySteinerPoints() const;
    
    // === Task 2.3: Steiner Point Merging ===
    void mergeSteinerPointsOnEdge(const std::pair<int, int>& edge_key);
    double computeIntervalSizeAt(const SteinerPoint& point, int source_vertex_id, const VertexGeometricParams& params);
    double findIntervalSizeCrossover(const VertexGeometricParams& params1, 
                                    const VertexGeometricParams& params2, 
                                    double edge_length);
    std::vector<SteinerPoint> getPointsOnEdge(const std::pair<int, int>& edge_key);
    void replacePointsOnEdge(const std::pair<int, int>& edge_key, const std::vector<SteinerPoint>& new_points);
    
    // === Task 3: Approximation Graph Construction Helpers ===
    std::vector<int> getVerticesOnFace(const HE_Face* face);
    std::vector<int> getSteinerPointsOnFace(const HE_Face* face);
    std::vector<int> getAllPointsOnFace(const HE_Face* face);
    bool createCompleteSubgraph(const HE_Face* face);
    
    // === Task 5: Sparse Graph Construction Helpers ===
    bool isSteinerPointNearVertex(int steiner_graph_id, int vertex_graph_id);
    int createGraphEdge(int vertex1_id, int vertex2_id, const HE_Face* face);
};

// === Fibonacci Heap Implementation for Dijkstra ===

/**
 * @struct FibonacciHeapNode
 * @brief Node in Fibonacci heap for optimal Dijkstra performance
 */
template<typename K, typename V>
struct FibonacciHeapNode {
    K key;
    V value;
    FibonacciHeapNode* parent = nullptr;
    FibonacciHeapNode* child = nullptr;
    FibonacciHeapNode* left = nullptr;
    FibonacciHeapNode* right = nullptr;
    int degree = 0;
    bool marked = false;
    
    FibonacciHeapNode(K k, V v) : key(k), value(v) {
        left = right = this;
    }
};

/**
 * @class FibonacciHeap
 * @brief Fibonacci heap for O(log n) decrease-key operations in Dijkstra
 */
template<typename K, typename V>
class FibonacciHeap {
private:
    using Node = FibonacciHeapNode<K, V>;
    Node* min_node = nullptr;
    int node_count = 0;
    
    void consolidate();
    void link(Node* y, Node* x);
    void cut(Node* x, Node* y);
    void cascading_cut(Node* y);
    
public:
    ~FibonacciHeap();
    
    Node* insert(K key, V value);
    Node* minimum();
    Node* extract_min();
    void decrease_key(Node* x, K new_key);
    void remove(Node* x);
    bool empty() const { return min_node == nullptr; }
    int size() const { return node_count; }
};

#endif // SHORTEST_PATH_LIB_H