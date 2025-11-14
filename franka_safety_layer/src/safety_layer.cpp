#include "../include/franka_safety_layer/safety_layer.hpp"


SafetyLayer::SafetyLayer()
{

    // 1. Define the Allowed Workspace (AABB)
    workspace_ = {
        Eigen::Vector3d(-0.5, -0.9, 0.4), // min_limits [x_min, y_min, z_min]
        Eigen::Vector3d( 0.8,  0.9, 1.2)  // max_limits [x_max, y_max, z_max]
    };
    
    // 2. Define Forbidden AABBs
    forbidden_blocks_ = 
    {
        {   // center cam
            Eigen::Vector3d(-0.5, -0.2, 0.0), 
            Eigen::Vector3d(-0.1, 0.2, 0.7)
        }, 
        {   // right cam
            Eigen::Vector3d(0.3, -0.9, 0.0), 
            Eigen::Vector3d(0.8, -0.65, 0.7)
        },
        {   // left cam 
            Eigen::Vector3d(0.3, 0.65, 0.0), 
            Eigen::Vector3d(0.8, 0.9, 0.7)
        },
        {   // drawer
            Eigen::Vector3d(0.5, -0.2, 0.0), 
            Eigen::Vector3d(0.8, 0.2, 0.6) 
        },
    };

    effective_workspace_ = {
        workspace_.min_limits + Eigen::Vector3d::Constant(safety_distance_),
        workspace_.max_limits - Eigen::Vector3d::Constant(safety_distance_)
    };


    std::cout << "SafetyLayer initialized with " << forbidden_blocks_.size() 
              << " forbidden AABBs and safety distance " << safety_distance_ << " m." << std::endl;
}

void SafetyLayer::init(Eigen::Vector3d &initial_position, rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub)
{
    initial_position_ = initial_position;
    vis_.init(marker_pub, workspace_, forbidden_blocks_);
}


Eigen::Vector3d SafetyLayer::closestPointToAABB(const AABB& box, const Eigen::Vector3d& query_point) const
{
    Eigen::Vector3d closest_point;
    for (int i = 0; i < 3; ++i)
    {
        closest_point[i] = std::max(box.min_limits[i], std::min(box.max_limits[i], query_point[i]));
    }
    return closest_point;
}


Eigen::Vector3d SafetyLayer::clampToAABB(const Eigen::Vector3d& position) const
{
    Eigen::Vector3d clamped_pos;
    for (int i = 0; i < 3; ++i)
    {
        clamped_pos[i] = std::max(effective_workspace_.min_limits[i], std::min(effective_workspace_.max_limits[i], position[i]));
    }           
    return clamped_pos;
}


Eigen::Vector3d SafetyLayer::calculatePushOff(const Eigen::Vector3d& current_position, const Eigen::Vector3d& robot_position) const
{
    double max_required_push_magnitude = 0.0;
    Eigen::Vector3d best_push_direction = Eigen::Vector3d::Zero();

    // Determine the preferred push direction towards the robot position
    Eigen::Vector3d push_preference_direction = robot_position - current_position;
    
    // Normalize preference direction, using a fallback if at the initial position
    if (push_preference_direction.norm() > 1e-9) {
        push_preference_direction.normalize();
    } else {
        push_preference_direction = Eigen::Vector3d(0.0, 0.0, 1.0); // Fallback: +Z
    }
    
    // Iterate over all forbidden blocks
    for (const auto& block : forbidden_blocks_)
    {
        // Define the limits of the inflated AABB (block + safety_distance_)
        Eigen::Vector3d inflated_min = block.min_limits - Eigen::Vector3d::Constant(safety_distance_);
        Eigen::Vector3d inflated_max = block.max_limits + Eigen::Vector3d::Constant(safety_distance_);

        // Check if the current_position is inside the inflated AABB
        bool is_inside_inflated = true;
        for (int i = 0; i < 3; ++i)
        {
            if (current_position[i] <= inflated_min[i] || current_position[i] >= inflated_max[i])
            {
                is_inside_inflated = false;
                break;
            }
        }

        if (is_inside_inflated)
        {
            std::cout << "target " << current_position.transpose() << " inside" << inflated_min.transpose() << ", " << inflated_max.transpose() << std::endl;

            double current_required_magnitude = std::numeric_limits<double>::infinity();
            Eigen::Vector3d current_push_direction = Eigen::Vector3d::Zero();

            // Find the required push magnitude and direction based on the 'push_preference_direction'
            for (int i = 0; i < 3; ++i)
            {
                double magnitude_i = 0.0;
                
                if (std::abs(push_preference_direction[i]) > 1e-6)
                {
                    double preferred_axis_direction = (push_preference_direction[i] > 0.0) ? 1.0 : -1.0;

                    if (preferred_axis_direction > 0.0)
                    {
                        // Push in the positive direction (exit via max face)
                        magnitude_i = inflated_max[i] - current_position[i];
                    }
                    else
                    {
                        // Push in the negative direction (exit via min face)
                        magnitude_i = current_position[i] - inflated_min[i];
                    }

                    // Track the most critical push for the current block
                    if (magnitude_i < current_required_magnitude)
                    {
                        current_required_magnitude = magnitude_i;
                        current_push_direction = Eigen::Vector3d::Zero();
                        current_push_direction[i] = preferred_axis_direction;
                    }
                }
            }
            
            // Update Best Push Vector based on max required magnitude across all blocks
            if (current_required_magnitude > max_required_push_magnitude)
            {
                max_required_push_magnitude = current_required_magnitude;
                best_push_direction = current_push_direction;
            }
        }
    }

    // Return the push vector (direction * magnitude)
    return best_push_direction * max_required_push_magnitude;
}
pinocchio::SE3 SafetyLayer::adjustToSafePose(const pinocchio::SE3& robot_pose, const pinocchio::SE3& desired_pose) const
{
    // 1. Extract desired position and orientation
    Eigen::Vector3d desired_position = desired_pose.translation();
    
    // initial clamp to not work unnessesarry with invalid positions.
    Eigen::Vector3d safe_position = clampToAABB(desired_position);

    const int MAX_ITERATIONS = 10;
    Eigen::Vector3d total_push_vector = Eigen::Vector3d::Zero();
    int push_iterations = 0;

    // --- Safety Check 1: Forbidden Blocks Avoidance (Iterative) ---
    for (int iter = 0; iter < MAX_ITERATIONS; ++iter)
    {
        Eigen::Vector3d current_push_off = calculatePushOff(safe_position, robot_pose.translation());

        
        if (current_push_off.norm() < 1e-6)
        {
            // Position is now safe from all blocks
            push_iterations = iter;
            break; 
        }
        else
        {
            std::cout << "iter " << iter << ", pos : " << safe_position.transpose() << std::endl;
            std::cout << "iter " << iter << ", push : " << current_push_off.transpose() << std::endl;
        }

        safe_position += current_push_off;
        total_push_vector += current_push_off;
        
        if (iter == MAX_ITERATIONS - 1) {
            std::cerr << "Warning: Maximum safety layer iterations (" << MAX_ITERATIONS 
                      << ") reached. Position still unsafe from blocks." << std::endl;
            push_iterations = MAX_ITERATIONS;
            // TODO safe fallback
        }
    }

    if(push_iterations > 0) {
        std::cout << "final : " << safe_position.transpose() << std::endl;
        std::cout << "dir " << (robot_pose.translation()- safe_position).transpose() << std::endl;

    }

    std::cout << "----------- " << std::endl;

    // 2. Allowed Workspace Clamping done (again) at the end to ensure the final position is within bounds, 
    Eigen::Vector3d final_safe_position = clampToAABB(safe_position);

    // 4. Return the new SE3 pose with the original orientation and the safe position
    return pinocchio::SE3(desired_pose.rotation(), final_safe_position);
}


double SafetyLayer::getShortestDistanceToSafetyBoundary(const Eigen::Vector3d& query_position) const
{
    // A positive 's' means we are safe. A negative 's' means we have penetrated the safety zone.
    double min_distance_remaining = std::numeric_limits<double>::infinity();

    // 1. Check distance to all forbidden AABBs
    for (const auto& block : forbidden_blocks_)
    {
        // Find the closest point on the forbidden block surface
        Eigen::Vector3d closest_on_block = closestPointToAABB(block, query_position);
        
        // Actual distance to the block surface
        double distance_to_block = (query_position - closest_on_block).norm();

        // Distance remaining until the safety boundary (distance to block minus safety_distance_)
        double remaining_distance_s = distance_to_block - safety_distance_;
        
        min_distance_remaining = std::min(min_distance_remaining, remaining_distance_s);
    }
    
    // 2. Consider the distance to the Allowed AABB boundary (Workspace limits)
    Eigen::Vector3d effective_min = workspace_.min_limits + Eigen::Vector3d::Constant(safety_distance_);
    Eigen::Vector3d effective_max = workspace_.max_limits - Eigen::Vector3d::Constant(safety_distance_);

    // Check if position is outside the effective safe workspace
    if (query_position.x() < effective_min.x() || query_position.x() > effective_max.x() ||
        query_position.y() < effective_min.y() || query_position.y() > effective_max.y() ||
        query_position.z() < effective_min.z() || query_position.z() > effective_max.z())
    {
        // If outside or on the boundary, distance 's' is 0 or negative (taken care of by forbidden blocks).
        min_distance_remaining = std::min(min_distance_remaining, 0.0);
    }
    else
    {
        // If inside the effective safe AABB, find the shortest distance to the boundary.
        double min_dist_to_workspace_face = std::numeric_limits<double>::infinity();
        for (int i = 0; i < 3; ++i)
        {
            // Distance to min effective face
            min_dist_to_workspace_face = std::min(min_dist_to_workspace_face, query_position[i] - effective_min[i]);
            
            // Distance to max effective face
            min_dist_to_workspace_face = std::min(min_dist_to_workspace_face, effective_max[i] - query_position[i]);
        }
        
        min_distance_remaining = std::min(min_distance_remaining, min_dist_to_workspace_face);
    }

    return min_distance_remaining;
}


double SafetyLayer::getMaxSafeVelocity(const Eigen::Vector3d& position) const
{
    // The closest distance 's' is the remaining distance to the safety boundary.
    double s = getShortestDistanceToSafetyBoundary(position);
    
    // safe_velocity = min(max_velocity, sqrt(2 * a * max(s, 0.0)))
    double safe_velocity = std::min(max_velocity_, min_velocity_ + std::sqrt(2 * safety_stopping_acceleration_ * std::max(s, 0.0)));
    
    return safe_velocity;
}