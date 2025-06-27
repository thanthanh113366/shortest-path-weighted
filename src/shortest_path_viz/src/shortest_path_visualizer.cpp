#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <limits>
#include <cmath>
#include "ShortestPathLib.h"

class ShortestPathVisualizer : public rclcpp::Node
{
public:
    ShortestPathVisualizer() : Node("shortest_path_visualizer")
    {
        // Publishers
        mesh_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "polyhedron_mesh", 10);
        steiner_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "steiner_points", 10);
        params_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "geometric_params", 10);
        graph_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "approximation_graph", 10);
        path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "shortest_path", 10);
        path_v2_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "shortest_path_v2", 10);
            
        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ShortestPathVisualizer::publish_visualization, this));
            
        // Load and process polyhedron
        setup_polyhedron();
        
        RCLCPP_INFO(this->get_logger(), "Shortest Path Visualizer started");
    }

private:
    void setup_polyhedron()
    {
        // Load cube mesh 
        if (!poly_.loadFromOFF("models/cube.off")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load models/cube.off");
            return;
        }
        
        // Compute geometric parameters
        double epsilon = 0.1;
        if (!poly_.computeGeometricParameters(epsilon)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute geometric parameters");
            return;
        }
        
        // Place Steiner points
        if (!poly_.placeSteinerPoints()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to place Steiner points");
            return;
        }
        
        // Get Steiner count before merging
        original_steiner_count_ = poly_.getSteinerPoints().size();
        
        // Merge Steiner points
        if (!poly_.mergeSteinerPoints()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to merge Steiner points");
            return;
        }
        
        // Get final Steiner count
        final_steiner_count_ = poly_.getSteinerPoints().size();
        
        // Build DENSE approximation graph first (Task 3)
        if (!poly_.buildApproximationGraph()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build approximation graph");
            return;
        }
        
        // Store dense graph statistics
        dense_edge_count_ = poly_.getGraphEdges().size();
        
        // Build PRUNED approximation graph (Task 5 - optimization)
        if (!poly_.buildPrunedApproximationGraph()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build pruned approximation graph");
            return;
        }
        
        // Store sparse graph statistics
        sparse_edge_count_ = poly_.getGraphEdges().size();
        edge_reduction_ratio_ = 100.0 * (dense_edge_count_ - sparse_edge_count_) / dense_edge_count_;
        
        // Compute shortest path (Task 4 - Version 1: Vertex-to-Vertex)
        shortest_path_result_ = poly_.findApproximateShortestPath(0, 5, epsilon);
        if (shortest_path_result_.path_found) {
            RCLCPP_INFO(this->get_logger(), "Version 1 shortest path computed successfully! Cost: %.6f", 
                       shortest_path_result_.weighted_cost);
        } else {
            RCLCPP_WARN(this->get_logger(), "Version 1 shortest path not found");
        }
        
        // Compute shortest path (Task 4 - Version 2: Paper-Compliant Surface Points)
        // Using barycentric coordinates on cube faces for guaranteed surface points
        
        // Point 1: On Face 0 (vertices 0,1,2) with barycentric (0.6, 0.3, 0.1)
        // Face 0: V0=(1,1,-1), V1=(1,-1,-1), V2=(-1,-1,-1), weight=1.0
        Vector3D source_point;
        source_point.x = 0.6*1.0 + 0.3*1.0 + 0.1*(-1.0);    // = 0.8
        source_point.y = 0.6*1.0 + 0.3*(-1.0) + 0.1*(-1.0); // = 0.2
        source_point.z = 0.6*(-1.0) + 0.3*(-1.0) + 0.1*(-1.0); // = -1.0
        
        // Point 2: On Face 2 (vertices 4,7,6) with barycentric (0.2, 0.5, 0.3)
        // Face 2: V4=(1,1,1), V7=(-1,1,1), V6=(-1,-1,1), weight=0.8
        Vector3D target_point;
        target_point.x = 0.2*1.0 + 0.5*(-1.0) + 0.3*(-1.0);  // = -0.6
        target_point.y = 0.2*1.0 + 0.5*1.0 + 0.3*(-1.0);     // = 0.4
        target_point.z = 0.2*1.0 + 0.5*1.0 + 0.3*1.0;        // = 1.0
        shortest_path_result_v2_ = poly_.findApproximateShortestPath(source_point, target_point, epsilon);
        if (shortest_path_result_v2_.path_found) {
            RCLCPP_INFO(this->get_logger(), "Version 2 shortest path computed successfully! Cost: %.6f", 
                       shortest_path_result_v2_.weighted_cost);
        } else {
            RCLCPP_WARN(this->get_logger(), "Version 2 shortest path not found");
        }
        
        RCLCPP_INFO(this->get_logger(), "Polyhedron setup complete:");
        RCLCPP_INFO(this->get_logger(), "  - Steiner points: %zu (reduced from %zu)", final_steiner_count_, original_steiner_count_);
        RCLCPP_INFO(this->get_logger(), "  - Dense graph: %zu edges", dense_edge_count_);
        RCLCPP_INFO(this->get_logger(), "  - Sparse graph: %zu edges (%.1f%% reduction)", sparse_edge_count_, edge_reduction_ratio_);
    }
    
    void publish_visualization()
    {
        publish_mesh();
        publish_steiner_points();
        publish_geometric_params();
        publish_approximation_graph();
        publish_shortest_path();
        publish_shortest_path_v2();
    }
    
    void publish_mesh()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Publish vertices as spheres
        auto vertex_marker = visualization_msgs::msg::Marker();
        vertex_marker.header.frame_id = "map";
        vertex_marker.header.stamp = this->now();
        vertex_marker.ns = "vertices";
        vertex_marker.id = 0;
        vertex_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        vertex_marker.action = visualization_msgs::msg::Marker::ADD;
        vertex_marker.scale.x = vertex_marker.scale.y = vertex_marker.scale.z = 0.1;
        vertex_marker.color.r = 1.0; vertex_marker.color.g = 0.0; 
        vertex_marker.color.b = 0.0; vertex_marker.color.a = 1.0;
        
        // Add vertex positions
        for (const auto& vertex : poly_.getVertices()) {
            geometry_msgs::msg::Point point;
            point.x = vertex->pos.x;
            point.y = vertex->pos.y; 
            point.z = vertex->pos.z;
            vertex_marker.points.push_back(point);
        }
        marker_array.markers.push_back(vertex_marker);
        
        // Publish edges as lines
        auto edge_marker = visualization_msgs::msg::Marker();
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = this->now();
        edge_marker.ns = "edges";
        edge_marker.id = 1;
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.scale.x = 0.02; // Line width
        edge_marker.color.r = 0.5; edge_marker.color.g = 0.5;
        edge_marker.color.b = 0.5; edge_marker.color.a = 1.0;
        
        // Add edge lines
        for (const auto& he : poly_.getHalfEdges()) {
            if (he->origin && he->next && he->next->origin) {
                geometry_msgs::msg::Point start, end;
                start.x = he->origin->pos.x;
                start.y = he->origin->pos.y;
                start.z = he->origin->pos.z;
                end.x = he->next->origin->pos.x;
                end.y = he->next->origin->pos.y;
                end.z = he->next->origin->pos.z;
                edge_marker.points.push_back(start);
                edge_marker.points.push_back(end);
            }
        }
        marker_array.markers.push_back(edge_marker);
        
        mesh_publisher_->publish(marker_array);
    }
    
    void publish_steiner_points()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        auto steiner_marker = visualization_msgs::msg::Marker();
        steiner_marker.header.frame_id = "map";
        steiner_marker.header.stamp = this->now();
        steiner_marker.ns = "steiner_points";
        steiner_marker.id = 0;
        steiner_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        steiner_marker.action = visualization_msgs::msg::Marker::ADD;
        steiner_marker.scale.x = steiner_marker.scale.y = steiner_marker.scale.z = 0.05;
        steiner_marker.color.r = 0.0; steiner_marker.color.g = 1.0;
        steiner_marker.color.b = 0.0; steiner_marker.color.a = 0.8;
        
        // Add Steiner point positions
        for (const auto& point : poly_.getSteinerPoints()) {
            geometry_msgs::msg::Point pos;
            pos.x = point.pos.x;
            pos.y = point.pos.y;
            pos.z = point.pos.z;
            steiner_marker.points.push_back(pos);
            
            // Color code by source vertex
            std_msgs::msg::ColorRGBA color;
            color.a = 0.8;
            switch (point.source_vertex_id % 8) {
                case 0: color.r = 1.0; color.g = 0.0; color.b = 0.0; break;
                case 1: color.r = 0.0; color.g = 1.0; color.b = 0.0; break;
                case 2: color.r = 0.0; color.g = 0.0; color.b = 1.0; break;
                case 3: color.r = 1.0; color.g = 1.0; color.b = 0.0; break;
                case 4: color.r = 1.0; color.g = 0.0; color.b = 1.0; break;
                case 5: color.r = 0.0; color.g = 1.0; color.b = 1.0; break;
                case 6: color.r = 1.0; color.g = 0.5; color.b = 0.0; break;
                case 7: color.r = 0.5; color.g = 0.0; color.b = 1.0; break;
            }
            steiner_marker.colors.push_back(color);
        }
        
        marker_array.markers.push_back(steiner_marker);
        steiner_publisher_->publish(marker_array);
    }
    
    void publish_geometric_params()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Add text labels for geometric parameters
        int id = 0;
        for (const auto& vertex : poly_.getVertices()) {
            const auto& params = poly_.getVertexGeometricParams(vertex->id);
            
            auto text_marker = visualization_msgs::msg::Marker();
            text_marker.header.frame_id = "map";
            text_marker.header.stamp = this->now();
            text_marker.ns = "vertex_params";
            text_marker.id = id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position.x = vertex->pos.x + 0.2;
            text_marker.pose.position.y = vertex->pos.y + 0.2;
            text_marker.pose.position.z = vertex->pos.z + 0.2;
            
            text_marker.scale.z = 0.1; // Text size
            text_marker.color.r = 1.0; text_marker.color.g = 1.0;
            text_marker.color.b = 1.0; text_marker.color.a = 1.0;
            
            char text_buffer[200];
            snprintf(text_buffer, sizeof(text_buffer), 
                "V%d\nh_v=%.3f\nθ_v=%.1f°\nr_v=%.3f\nδ=%.3f",
                vertex->id, params.h_v, params.theta_v * 180.0 / M_PI, 
                params.r_v, params.delta);
            text_marker.text = std::string(text_buffer);
            
            marker_array.markers.push_back(text_marker);
        }
        
        params_publisher_->publish(marker_array);
    }
    
    void publish_approximation_graph()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Visualize graph edges with different colors based on weight
        auto graph_edges_marker = visualization_msgs::msg::Marker();
        graph_edges_marker.header.frame_id = "map";
        graph_edges_marker.header.stamp = this->now();
        graph_edges_marker.ns = "graph_edges";
        graph_edges_marker.id = 0;
        graph_edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        graph_edges_marker.action = visualization_msgs::msg::Marker::ADD;
        graph_edges_marker.scale.x = 0.005; // Thin lines for graph edges
        
        // Get graph data
        const auto& graph_vertices = poly_.getGraphVertices();
        const auto& graph_edges = poly_.getGraphEdges();
        
        // Find min/max edge weights for color mapping
        double min_weight = 1e10, max_weight = 0.0;
        for (const auto& edge : graph_edges) {
            min_weight = std::min(min_weight, edge.weight);
            max_weight = std::max(max_weight, edge.weight);
        }
        
        // Limit visualization to avoid overwhelming RViz (show subset of edges)
        int max_edges_to_show = 2000; // Limit for performance
        int edge_step = std::max(1, (int)graph_edges.size() / max_edges_to_show);
        
        for (size_t i = 0; i < graph_edges.size(); i += edge_step) {
            const auto& edge = graph_edges[i];
            
            // Find positions of the two vertices
            Vector3D pos1, pos2;
            bool found1 = false, found2 = false;
            
            for (const auto& gv : graph_vertices) {
                if (gv.id == edge.vertex1_id) {
                    pos1 = gv.pos;
                    found1 = true;
                }
                if (gv.id == edge.vertex2_id) {
                    pos2 = gv.pos;
                    found2 = true;
                }
                if (found1 && found2) break;
            }
            
            if (found1 && found2) {
                geometry_msgs::msg::Point start, end;
                start.x = pos1.x; start.y = pos1.y; start.z = pos1.z;
                end.x = pos2.x; end.y = pos2.y; end.z = pos2.z;
                
                graph_edges_marker.points.push_back(start);
                graph_edges_marker.points.push_back(end);
                
                // Color based on weight (blue = light weight, red = heavy weight)
                std_msgs::msg::ColorRGBA color1, color2;
                double normalized_weight = (edge.weight - min_weight) / (max_weight - min_weight);
                
                color1.r = color2.r = normalized_weight;
                color1.g = color2.g = 0.0;
                color1.b = color2.b = 1.0 - normalized_weight;
                color1.a = color2.a = 0.3; // Semi-transparent
                
                graph_edges_marker.colors.push_back(color1);
                graph_edges_marker.colors.push_back(color2);
            }
        }
        
        marker_array.markers.push_back(graph_edges_marker);
        
        // Visualize graph vertices with different symbols for original vs Steiner points
        auto graph_vertices_marker = visualization_msgs::msg::Marker();
        graph_vertices_marker.header.frame_id = "map";
        graph_vertices_marker.header.stamp = this->now();
        graph_vertices_marker.ns = "graph_vertices";
        graph_vertices_marker.id = 1;
        graph_vertices_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        graph_vertices_marker.action = visualization_msgs::msg::Marker::ADD;
        graph_vertices_marker.scale.x = graph_vertices_marker.scale.y = graph_vertices_marker.scale.z = 0.03;
        
        for (const auto& gv : graph_vertices) {
            geometry_msgs::msg::Point point;
            point.x = gv.pos.x;
            point.y = gv.pos.y;
            point.z = gv.pos.z;
            graph_vertices_marker.points.push_back(point);
            
            // Color code: Original vertices = bright white, Steiner points = cyan
            std_msgs::msg::ColorRGBA color;
            if (gv.is_original_vertex) {
                color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = 1.0; // White for original
            } else {
                color.r = 0.0; color.g = 0.8; color.b = 0.8; color.a = 0.8; // Cyan for Steiner
            }
            graph_vertices_marker.colors.push_back(color);
        }
        
        marker_array.markers.push_back(graph_vertices_marker);
        
        // Add text info about graph statistics
        auto stats_marker = visualization_msgs::msg::Marker();
        stats_marker.header.frame_id = "map";
        stats_marker.header.stamp = this->now();
        stats_marker.ns = "graph_stats";
        stats_marker.id = 2;
        stats_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        stats_marker.action = visualization_msgs::msg::Marker::ADD;
        
        stats_marker.pose.position.x = -2.0;
        stats_marker.pose.position.y = -2.0;
        stats_marker.pose.position.z = 3.0;
        
        stats_marker.scale.z = 0.2; // Text size
        stats_marker.color.r = 1.0; stats_marker.color.g = 1.0;
        stats_marker.color.b = 0.0; stats_marker.color.a = 1.0; // Yellow text
        
        char stats_buffer[400];
        snprintf(stats_buffer, sizeof(stats_buffer), 
            "Pruned Approximation Graph G* (Task 5)\nVertices: %zu\nSteiner: %zu (reduced from %zu)\nDense edges: %zu → Sparse edges: %zu\nReduction: %.1f%%\nWeight range: %.2f - %.2f\nShowing: %d edges",
            graph_vertices.size(), final_steiner_count_, original_steiner_count_,
            dense_edge_count_, sparse_edge_count_, edge_reduction_ratio_,
            min_weight, max_weight, std::min((int)graph_edges.size(), max_edges_to_show));
        stats_marker.text = std::string(stats_buffer);
        
        marker_array.markers.push_back(stats_marker);
        
        graph_publisher_->publish(marker_array);
    }
    
    void publish_shortest_path()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        if (!shortest_path_result_.path_found) {
            // Clear any existing path markers
            auto clear_marker = visualization_msgs::msg::Marker();
            clear_marker.header.frame_id = "map";
            clear_marker.header.stamp = this->now();
            clear_marker.ns = "shortest_path";
            clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(clear_marker);
            path_publisher_->publish(marker_array);
            return;
        }
        
        // Visualize path as thick lines
        auto path_marker = visualization_msgs::msg::Marker();
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = "shortest_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.08; // Thick line
        path_marker.color.r = 1.0; path_marker.color.g = 0.0; 
        path_marker.color.b = 1.0; path_marker.color.a = 1.0; // Magenta path
        
        // Add path points
        for (const auto& pos : shortest_path_result_.path) {
            geometry_msgs::msg::Point point;
            point.x = pos.x;
            point.y = pos.y;
            point.z = pos.z;
            path_marker.points.push_back(point);
        }
        marker_array.markers.push_back(path_marker);
        
        // Visualize path vertices as larger spheres
        auto path_vertices_marker = visualization_msgs::msg::Marker();
        path_vertices_marker.header.frame_id = "map";
        path_vertices_marker.header.stamp = this->now();
        path_vertices_marker.ns = "path_vertices";
        path_vertices_marker.id = 1;
        path_vertices_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        path_vertices_marker.action = visualization_msgs::msg::Marker::ADD;
        path_vertices_marker.scale.x = path_vertices_marker.scale.y = path_vertices_marker.scale.z = 0.12;
        
        for (size_t i = 0; i < shortest_path_result_.path.size(); ++i) {
            const auto& pos = shortest_path_result_.path[i];
            geometry_msgs::msg::Point point;
            point.x = pos.x;
            point.y = pos.y;
            point.z = pos.z;
            path_vertices_marker.points.push_back(point);
            
            // Color code: start = green, end = red, intermediate = yellow
            std_msgs::msg::ColorRGBA color;
            if (i == 0) {
                color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 1.0; // Green start
            } else if (i == shortest_path_result_.path.size() - 1) {
                color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0; // Red end
            } else {
                color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 1.0; // Yellow intermediate
            }
            path_vertices_marker.colors.push_back(color);
        }
        marker_array.markers.push_back(path_vertices_marker);
        
        // Add text info about shortest path
        auto path_info_marker = visualization_msgs::msg::Marker();
        path_info_marker.header.frame_id = "map";
        path_info_marker.header.stamp = this->now();
        path_info_marker.ns = "path_info";
        path_info_marker.id = 2;
        path_info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        path_info_marker.action = visualization_msgs::msg::Marker::ADD;
        
        path_info_marker.pose.position.x = -2.0;
        path_info_marker.pose.position.y = -2.0;
        path_info_marker.pose.position.z = 4.0;
        
        path_info_marker.scale.z = 0.2; // Text size
        path_info_marker.color.r = 1.0; path_info_marker.color.g = 0.0;
        path_info_marker.color.b = 1.0; path_info_marker.color.a = 1.0; // Magenta text
        
        char path_buffer[350];
        snprintf(path_buffer, sizeof(path_buffer), 
            "Method 1: Vertex-to-Vertex (Task 4)\nFound: YES\nStart: V0 → End: V5\nWeighted Cost: %.6f\nPath Points: %zu\nEpsilon: 0.1 (≤10%% error)",
            shortest_path_result_.weighted_cost, shortest_path_result_.path.size());
        path_info_marker.text = std::string(path_buffer);
        
        marker_array.markers.push_back(path_info_marker);
        
        path_publisher_->publish(marker_array);
    }
    
    void publish_shortest_path_v2()
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        if (!shortest_path_result_v2_.path_found) {
            // Clear any existing path markers
            auto clear_marker = visualization_msgs::msg::Marker();
            clear_marker.header.frame_id = "map";
            clear_marker.header.stamp = this->now();
            clear_marker.ns = "shortest_path_v2";
            clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(clear_marker);
            path_v2_publisher_->publish(marker_array);
            return;
        }

        // === Solution 1: Surface Projection Path Rendering ===
        publish_face_based_path(marker_array, shortest_path_result_v2_.path, "shortest_path_v2");
        
        // Visualize path vertices as larger spheres
        auto path_vertices_marker = visualization_msgs::msg::Marker();
        path_vertices_marker.header.frame_id = "map";
        path_vertices_marker.header.stamp = this->now();
        path_vertices_marker.ns = "path_vertices_v2";
        path_vertices_marker.id = 1;
        path_vertices_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        path_vertices_marker.action = visualization_msgs::msg::Marker::ADD;
        path_vertices_marker.scale.x = path_vertices_marker.scale.y = path_vertices_marker.scale.z = 0.15;
        
        for (size_t i = 0; i < shortest_path_result_v2_.path.size(); ++i) {
            const auto& pos = shortest_path_result_v2_.path[i];
            geometry_msgs::msg::Point point;
            point.x = pos.x;
            point.y = pos.y;
            point.z = pos.z;
            path_vertices_marker.points.push_back(point);
            
            // Color code: start = bright green, end = bright red, intermediate = orange
            std_msgs::msg::ColorRGBA color;
            if (i == 0) {
                color.r = 0.2; color.g = 1.0; color.b = 0.2; color.a = 1.0; // Bright green start
            } else if (i == shortest_path_result_v2_.path.size() - 1) {
                color.r = 1.0; color.g = 0.2; color.b = 0.2; color.a = 1.0; // Bright red end
            } else {
                color.r = 1.0; color.g = 0.6; color.b = 0.0; color.a = 1.0; // Orange intermediate
            }
            path_vertices_marker.colors.push_back(color);
        }
        marker_array.markers.push_back(path_vertices_marker);
        
        // Add text info about shortest path v2 (paper-compliant)
        auto path_info_marker = visualization_msgs::msg::Marker();
        path_info_marker.header.frame_id = "map";
        path_info_marker.header.stamp = this->now();
        path_info_marker.ns = "path_info_v2";
        path_info_marker.id = 2;
        path_info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        path_info_marker.action = visualization_msgs::msg::Marker::ADD;
        
        path_info_marker.pose.position.x = -2.0;
        path_info_marker.pose.position.y = -2.0;
        path_info_marker.pose.position.z = 4.5;
        
        path_info_marker.scale.z = 0.2; // Text size
        path_info_marker.color.r = 0.0; path_info_marker.color.g = 1.0;
        path_info_marker.color.b = 1.0; path_info_marker.color.a = 1.0; // Cyan text
        
        char path_buffer[600];
        snprintf(path_buffer, sizeof(path_buffer), 
            "Method 2: Surface Points (Paper-Compliant)\nFound: YES\nStart: (0.800,0.200,-1.000) Face0 → End: (-0.600,0.400,1.000) Face2\nWeighted Cost: %.6f\nPath Points: %zu\nTemp vertices added/removed dynamically\nEpsilon: 0.1 (≤10%% error)",
            shortest_path_result_v2_.weighted_cost, shortest_path_result_v2_.path.size());
        path_info_marker.text = std::string(path_buffer);
        
        marker_array.markers.push_back(path_info_marker);
        
        path_v2_publisher_->publish(marker_array);
    }

    // === Solution 1: Surface Projection Path Rendering ===
    void publish_face_based_path(visualization_msgs::msg::MarkerArray& marker_array, 
                                const std::vector<Vector3D>& path, 
                                const std::string& namespace_prefix)
    {
        if (path.size() < 2) return;
        
        RCLCPP_INFO(this->get_logger(), "=== Surface Projection Path Rendering ===");
        RCLCPP_INFO(this->get_logger(), "Path has %zu points", path.size());
        
        auto path_marker = visualization_msgs::msg::Marker();
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = this->now();
        path_marker.ns = namespace_prefix + "_surface_path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.15; // Thick line for visibility
        
        // Bright yellow for surface-projected path
        path_marker.color.r = 1.0; path_marker.color.g = 1.0; 
        path_marker.color.b = 0.0; path_marker.color.a = 1.0;
        
        // Create dense interpolated points along surface
        for (size_t i = 0; i < path.size() - 1; ++i) {
            const Vector3D& start = path[i];
            const Vector3D& end = path[i + 1];
            
            // Add start point
            geometry_msgs::msg::Point start_point;
            start_point.x = start.x; start_point.y = start.y; start_point.z = start.z;
            path_marker.points.push_back(start_point);
            
            // Interpolate multiple points between start and end
            int num_interpolations = 20; // Dense sampling for smooth curve
            for (int j = 1; j < num_interpolations; ++j) {
                double t = (double)j / num_interpolations;
                
                // Linear interpolation in 3D space
                Vector3D lerp_point;
                lerp_point.x = start.x + t * (end.x - start.x);
                lerp_point.y = start.y + t * (end.y - start.y);
                lerp_point.z = start.z + t * (end.z - start.z);
                
                // Project to nearest surface point
                Vector3D surface_point = projectToNearestSurface(lerp_point);
                
                geometry_msgs::msg::Point interp_point;
                interp_point.x = surface_point.x;
                interp_point.y = surface_point.y; 
                interp_point.z = surface_point.z;
                path_marker.points.push_back(interp_point);
            }
        }
        
        // Add final point
        const Vector3D& final = path.back();
        geometry_msgs::msg::Point final_point;
        final_point.x = final.x; final_point.y = final.y; final_point.z = final.z;
        path_marker.points.push_back(final_point);
        
        marker_array.markers.push_back(path_marker);
        
        RCLCPP_INFO(this->get_logger(), "Rendered surface-projected path with %zu interpolated points", 
                   path_marker.points.size());
    }
    
    // Project point to nearest surface of polyhedron
    Vector3D projectToNearestSurface(const Vector3D& point) 
    {
        double min_distance = std::numeric_limits<double>::max();
        Vector3D closest_surface_point = point;
        
        // Check all faces to find closest surface point
        for (const auto& face : poly_.getFaces()) {
            Vector3D projected = projectPointToTriangle(point, face.get());
            double dist = distance(point, projected);
            
            if (dist < min_distance) {
                min_distance = dist;
                closest_surface_point = projected;
            }
        }
        
        return closest_surface_point;
    }
    
    // Project point to triangle face
    Vector3D projectPointToTriangle(const Vector3D& point, const HE_Face* face) 
    {
        // Get triangle vertices
        auto vertices = getTriangleVertices(face);
        const Vector3D& v0 = vertices[0];
        const Vector3D& v1 = vertices[1]; 
        const Vector3D& v2 = vertices[2];
        
        // Compute triangle normal
        Vector3D normal = crossProduct(subtract(v1, v0), subtract(v2, v0));
        normal = normalize(normal);
        
        // Project point to plane
        Vector3D v0_to_point = subtract(point, v0);
        double dist_to_plane = dotProduct(v0_to_point, normal);
        Vector3D plane_point = subtract(point, scale(normal, dist_to_plane));
        
        // Check if projection is inside triangle using barycentric coordinates
        Vector3D barycentric = computeBarycentric(plane_point, v0, v1, v2);
        
        // Clamp to triangle if outside
        if (barycentric.x < 0.0 || barycentric.y < 0.0 || barycentric.z < 0.0) {
            
            // Find closest edge/vertex
            Vector3D edge_projections[3] = {
                projectPointToLineSegment(point, v0, v1),
                projectPointToLineSegment(point, v1, v2),
                projectPointToLineSegment(point, v2, v0)
            };
            
            double min_dist = std::numeric_limits<double>::max();
            Vector3D closest = plane_point;
            
            for (int i = 0; i < 3; ++i) {
                double d = distance(point, edge_projections[i]);
                if (d < min_dist) {
                    min_dist = d;
                    closest = edge_projections[i];
                }
            }
            
            return closest;
        }
        
        return plane_point;
    }
    
    // Helper functions for vector operations
    Vector3D crossProduct(const Vector3D& a, const Vector3D& b) {
        return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
    }
    
    Vector3D subtract(const Vector3D& a, const Vector3D& b) {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }
    
    Vector3D scale(const Vector3D& v, double s) {
        return {v.x * s, v.y * s, v.z * s};
    }
    
    Vector3D normalize(const Vector3D& v) {
        double len = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
        if (len < 1e-10) return {0, 0, 0};
        return {v.x/len, v.y/len, v.z/len};
    }
    
    double dotProduct(const Vector3D& a, const Vector3D& b) {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }
    
    double distance(const Vector3D& a, const Vector3D& b) {
        Vector3D diff = subtract(a, b);
        return std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
    }
    
    Vector3D computeBarycentric(const Vector3D& p, const Vector3D& v0, const Vector3D& v1, const Vector3D& v2) {
        Vector3D v0v1 = subtract(v1, v0);
        Vector3D v0v2 = subtract(v2, v0);
        Vector3D v0p = subtract(p, v0);
        
        double dot00 = dotProduct(v0v2, v0v2);
        double dot01 = dotProduct(v0v2, v0v1);
        double dot02 = dotProduct(v0v2, v0p);
        double dot11 = dotProduct(v0v1, v0v1);
        double dot12 = dotProduct(v0v1, v0p);
        
        double inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
        double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;
        
        return {1.0 - u - v, v, u}; // barycentric coordinates (w0, w1, w2)
    }
    
    Vector3D projectPointToLineSegment(const Vector3D& point, const Vector3D& a, const Vector3D& b) {
        Vector3D ab = subtract(b, a);
        Vector3D ap = subtract(point, a);
        double ab_len_sq = dotProduct(ab, ab);
        
        if (ab_len_sq < 1e-10) return a; // Degenerate case
        
        double t = dotProduct(ap, ab) / ab_len_sq;
        t = std::max(0.0, std::min(1.0, t)); // Clamp to [0,1]
        
        return {a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z};
    }
    
    std::vector<Vector3D> getTriangleVertices(const HE_Face* face) {
        std::vector<Vector3D> vertices;
        HE_HalfEdge* he = face->edge;  // Use 'edge' not 'boundary'
        
        for (int i = 0; i < 3; ++i) {
            Vector3D vertex_pos;
            vertex_pos.x = he->origin->pos.x;  // Use 'pos.x' not 'x'
            vertex_pos.y = he->origin->pos.y;  // Use 'pos.y' not 'y'  
            vertex_pos.z = he->origin->pos.z;  // Use 'pos.z' not 'z'
            vertices.push_back(vertex_pos);
            he = he->next;
        }
        
        return vertices;
    }

    Polyhedron poly_;
    ShortestPathResult shortest_path_result_;
    ShortestPathResult shortest_path_result_v2_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr steiner_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr params_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_v2_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t original_steiner_count_;
    size_t final_steiner_count_;
    size_t dense_edge_count_;
    size_t sparse_edge_count_;
    double edge_reduction_ratio_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShortestPathVisualizer>());
    rclcpp::shutdown();
    return 0;
} 