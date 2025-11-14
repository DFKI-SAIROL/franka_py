#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Core>
#include <vector>
#include <tuple>

/**
 * @brief Represents an Axis-Aligned Bounding Box (AABB). Used for both forbidden regions and the allowed workspace.
 */
struct AABB
{
    Eigen::Vector3d min_limits; // [x_min, y_min, z_min]
    Eigen::Vector3d max_limits; // [x_max, y_max, z_max]
};


class WorkspaceVisualizer
{
public:
    WorkspaceVisualizer();

    void init(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub, AABB &workspace, std::vector<AABB> &forbidden_blocks);

    void publish_markers();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    AABB workspace_;
    std::vector<AABB> forbidden_blocks_;
    int marker_id_counter_ = 0;
    
    const double PLANE_THICKNESS = 0.005;

    void publish_workspace_planes();

    void publish_workspace_edges();

    visualization_msgs::msg::Marker create_volume_marker(const AABB& aabb);

};

