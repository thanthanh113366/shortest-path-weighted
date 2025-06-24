#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
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
        if (!poly_.loadFromOFF("models/icosahedron.off")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load models/icosahedron.off");
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
        
        // Merge Steiner points
        if (!poly_.mergeSteinerPoints()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to merge Steiner points");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Polyhedron setup complete");
    }
    
    void publish_visualization()
    {
        publish_mesh();
        publish_steiner_points();
        publish_geometric_params();
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
    
    Polyhedron poly_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mesh_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr steiner_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr params_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShortestPathVisualizer>());
    rclcpp::shutdown();
    return 0;
} 