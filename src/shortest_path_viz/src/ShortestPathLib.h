#ifndef SHORTEST_PATH_LIB_H
#define SHORTEST_PATH_LIB_H

#include <vector>
#include <string>
#include <map>
#include <utility> // For std::pair
#include <memory>  // For std::unique_ptr

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
 * @struct ShortestPathResult
 * @brief Contains the result of the shortest path finding algorithm.
 */
struct ShortestPathResult {
    bool path_found = false;    // Flag indicating whether a path was found successfully
    double weighted_cost = 0.0; // Total (weighted) cost of the path
    std::vector<Vector3D> path; // List of 3D points forming the
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
    Polyhedron() : next_steiner_id_(0) {}
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

private:
    // === Core Data Structures ===
    std::vector<std::unique_ptr<HE_Vertex>> vertices_;
    std::vector<std::unique_ptr<HE_Face>> faces_;
    std::vector<std::unique_ptr<HE_HalfEdge>> halfEdges_;
    std::vector<VertexGeometricParams> vertex_params_;  // Task 2.1: Geometric parameters per vertex
    std::vector<SteinerPoint> steiner_points_;         // Task 2.2-2.3: Steiner points on edges
    int next_steiner_id_;                              // Counter for unique Steiner point IDs
    
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
    bool shouldEliminatePoint(const SteinerPoint& point, const std::vector<SteinerPoint>& other_progression);
    std::vector<SteinerPoint> getPointsOnEdge(const std::pair<int, int>& edge_key);
    void replacePointsOnEdge(const std::pair<int, int>& edge_key, const std::vector<SteinerPoint>& new_points);
};

#endif // SHORTEST_PATH_LIB_H