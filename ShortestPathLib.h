#ifndef SHORTEST_PATH_LIB_H
#define SHORTEST_PATH_LIB_H

#include <vector>
#include <string>
#include <map>
#include <utility> // For std::pair
#include <memory>  // For std::unique_ptr

/**
 * @file ShortestPathLib.h
 * @brief A library for computing epsilon-approximate shortest paths on polyhedral surfaces.
 * Stage 1: Data structure definitions (using Half-Edge) and model loading.
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
 * @brief A container class for the entire polyhedron, built using the Half-Edge structure.
 * Manages memory for all components.
 */
class Polyhedron {
public:
    Polyhedron() = default;
    ~Polyhedron();

    // Disable copy semantics
    Polyhedron(const Polyhedron&) = delete;
    Polyhedron& operator=(const Polyhedron&) = delete;

    // The main loading function that builds the half-edge topology.
    bool loadFromOFF(const std::string& filename);

    // Accessors
    const std::vector<std::unique_ptr<HE_Vertex>>& getVertices() const { return vertices_; }
    const std::vector<std::unique_ptr<HE_Face>>& getFaces() const { return faces_; }
    const std::vector<std::unique_ptr<HE_HalfEdge>>& getHalfEdges() const { return halfEdges_; }

    /**
     * @brief Computes geometric parameters for all vertices (Task 2.1 from paper)
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
    std::vector<std::unique_ptr<HE_Vertex>> vertices_;
    std::vector<std::unique_ptr<HE_Face>> faces_;
    std::vector<std::unique_ptr<HE_HalfEdge>> halfEdges_;
    std::vector<VertexGeometricParams> vertex_params_;  // Task 2.1: Geometric parameters per vertex
    
    // Helper functions for paper-compliant validation
    bool validatePolyhedronRequirements();
    bool isTriangularFace(const HE_Face* face);
    double computeMinimumAngle(const HE_Face* face);
    double computeVertexHeight(const HE_Vertex* vertex);
    
    // Task 2.1: Geometric parameter computation functions
    double computeVertexHeight_v2(const HE_Vertex* vertex);      // Improved h_v computation
    double computeVertexMinAngle(const HE_Vertex* vertex);       // θ_v computation
    void displayGeometricParameters() const;
};

#endif // SHORTEST_PATH_LIB_H