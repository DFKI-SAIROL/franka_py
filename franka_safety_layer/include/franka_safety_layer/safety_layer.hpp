#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // for std::max, std::min
#include <limits>    // for std::numeric_limits

// Pinocchio/Eigen types for 3D pose and vectors
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Dense>

#include "vis.hpp"

/**
 * @brief Safety layer class to adjust desired Cartesian poses and limit velocity.
 *
 * This layer ensures the target position is:
 * 1. At least 'safety_distance' away from any forbidden AABB (enforced iteratively).
 * 2. Clamped within the defined AllowedAABB workspace.
 * It also computes a safe maximum velocity.
 */
class SafetyLayer
{
public:
    /**
     * @brief Constructor for the SafetyLayer.
     */
    SafetyLayer();

    void init(Eigen::Vector3d &initial_position, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub);


    /**
     * @brief Adjusts the desired Cartesian pose to a safe pose by applying
     * iterative collision avoidance and workspace clamping on the position component.
     */
    pinocchio::SE3 adjustToSafePose(const pinocchio::SE3& robot_pose, const pinocchio::SE3& desired_pose) const;

    double getMaxSafeVelocity(const Eigen::Vector3d& position) const;

    WorkspaceVisualizer vis_;

private:

    AABB workspace_, effective_workspace_;
    std::vector<AABB> forbidden_blocks_;
    Eigen::Vector3d initial_position_;

    double safety_distance_ = 0.05;
    double min_velocity_ = 0.1;
    double max_velocity_ = 10.0;
    double safety_stopping_acceleration_ = 5.0;

    /**
     * @brief Calculates the closest point on the AABB to the given query point.
     */
    Eigen::Vector3d closestPointToAABB(const AABB& box, const Eigen::Vector3d& query_point) const;

    /**
     * @brief Clamps the given position vector within the allowed AABB.
     */
    Eigen::Vector3d clampToAABB(const Eigen::Vector3d& position) const;

    /**
     * @brief Calculates the necessary push-off vector from the single most critical violation.
     * This function is intended to be called iteratively.
     * Returns Eigen::Vector3d::Zero() if no violation is found.
     */
    Eigen::Vector3d calculatePushOff(const Eigen::Vector3d& current_position, const Eigen::Vector3d& robot_position) const;
    
    double getShortestDistanceToSafetyBoundary(const Eigen::Vector3d& query_position) const;
    
};

