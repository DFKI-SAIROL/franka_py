#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // for std::max, std::min
#include <limits>    // for std::numeric_limits

#include <Eigen/Dense>

// Pinocchio Includes
#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>

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

    double getMaxSafeVelocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);

    WorkspaceVisualizer vis_;

    double current_distance_to_obstacle;
    double current_distance_to_obstacle_along_velocity_direction;

    bool other_initialized_ = false;
    Eigen::VectorXd other_q_;
    std::string other_urdf_decription_;
    std::string other_prefix_;
    pinocchio::Model other_model_;
    std::unique_ptr<pinocchio::Data> other_data_;
    pinocchio::FrameIndex other_ee_frame_id_;

    void init_other();
   
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
    double getDistanceAlongVelocity(const Eigen::Vector3d& query_position, const Eigen::Vector3d& query_velocity) const;
    
};

