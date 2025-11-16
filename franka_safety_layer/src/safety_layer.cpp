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

void SafetyLayer::init_other()
{
    if(other_urdf_decription_.empty()) return;

    // Load Pinocchio model from URDF string
    try {
        pinocchio::urdf::buildModelFromXML(other_urdf_decription_, other_model_);
        other_data_ = std::make_unique<pinocchio::Data>(other_model_);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from parameter: %s", e.what());
        return;
    }

    std::string end_effector_link_ = other_prefix_ + "fr3_link8";

    if (!other_model_.existFrame(end_effector_link_)) {
        RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", end_effector_link_.c_str());
        return;
    }

    other_ee_frame_id_ = other_model_.getFrameId(end_effector_link_);

    other_initialized_ = true;
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

    // Vector from current position to robot
    Eigen::Vector3d position_to_robot = robot_position - current_position;
    
    Eigen::Vector3d normalized_dir = Eigen::Vector3d::Zero();
    if (position_to_robot.norm() > 1e-9) {
        normalized_dir = position_to_robot.normalized();
    } else {
        normalized_dir = Eigen::Vector3d(0.0, 0.0, 1.0); // Fallback: +Z
    }
    
    // Iterate over all forbidden blocks
    for (const auto& block : forbidden_blocks_)
    {
        // Define the limits of the inflated AABB
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
            double current_dist_by_norm_direction = std::numeric_limits<double>::infinity();
            Eigen::Vector3d current_push_direction = Eigen::Vector3d::Zero();

            // Find the critical face that position_to_robot 'wants' to cross
            for (int i = 0; i < 3; ++i)
            {
                
                if (std::abs(normalized_dir[i]) > 1e-6)
                {
                    double magnitude_i;
                    double dist_by_norm_direction;
                    
                    if (normalized_dir[i] > 0.0)
                    {
                        // Exiting via MAX-face. Push direction is +1 on this axis.
                        magnitude_i = inflated_max[i] - current_position[i];
                        dist_by_norm_direction = std::abs(magnitude_i / normalized_dir[i]);
                        current_push_direction = Eigen::Vector3d::Zero();
                        current_push_direction[i] = 1.0; 
                    }
                    else // face_direction < 0.0
                    {
                        // Exiting via MIN-face. Push direction is -1 on this axis.
                        magnitude_i = current_position[i] - inflated_min[i];
                        dist_by_norm_direction = std::abs(magnitude_i / normalized_dir[i]);
                        current_push_direction = Eigen::Vector3d::Zero();
                        current_push_direction[i] = -1.0;
                    }

                    // Track the axis with the smallest distance (most critical face)
                    if (dist_by_norm_direction < current_dist_by_norm_direction)
                    {
                        current_dist_by_norm_direction = dist_by_norm_direction;
                        current_required_magnitude = magnitude_i;
                        best_push_direction = current_push_direction; 
                    }
                }
            }
            
            // Update Best Push Vector based on max required magnitude across all blocks
            if (current_required_magnitude < std::numeric_limits<double>::infinity() && current_required_magnitude > max_required_push_magnitude)
            {
                max_required_push_magnitude = current_required_magnitude;
                // best_push_direction is already set in the loop above
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
    double min_distance_remaining = 10;

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
        min_distance_remaining = 0;
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

double SafetyLayer::getDistanceAlongVelocity(const Eigen::Vector3d& query_position, const Eigen::Vector3d& query_velocity) const
{
    // Die kleinste Distanz entlang des Geschwindigkeitsvektors bis zum Auftreffen auf eine Grenze
    double min_distance_t = 10;

    // Normalisiere die Geschwindigkeit, um die Richtung des Strahls zu erhalten
    double velocity_norm = query_velocity.norm();
    
    // Wenn die Geschwindigkeit Null ist, gibt es keine Richtung, entlang derer geprüft werden kann.
    if (velocity_norm < 1e-9) {
        // Im Ruhezustand geben wir die Distanz zur nächstgelegenen Sicherheitsgrenze zurück.
        // Wir verwenden dafür die alte Logik (nur AABBs, da wir die Kollisionszeit wollen).
        return getShortestDistanceToSafetyBoundary(query_position); 
    }
    
    // Der Richtungsvektor des Strahls (Einheitsvektor)
    const Eigen::Vector3d ray_direction = query_velocity / velocity_norm;

    // --- 1. Distanz zu allen verbotenen AABBs (Forbidden Blocks + Safety Margin) ---
    
    for (const auto& block : forbidden_blocks_)
    {

        // Strahl-AABB-Schnittprüfung (Slab Method)
        double t_min = 0.0; // Minimaler positiver Schnittpunkt
        double t_max = std::numeric_limits<double>::infinity(); // Maximaler Schnittpunkt

        for (int i = 0; i < 3; ++i)
        {
            if (std::abs(ray_direction[i]) < 1e-6) // Strahl ist parallel zur Achse i
            {
                // Prüfen, ob der Startpunkt innerhalb der AABB-Grenzen dieser Achse liegt
                if (query_position[i] < block.min_limits[i] || query_position[i] > block.max_limits[i])
                {
                    // Der Strahl ist parallel zur Fläche und liegt außerhalb der Platte -> Kein Schnitt
                    t_min = std::numeric_limits<double>::infinity(); 
                    break; // Beende die Schleife, da kein Schnitt möglich ist
                }
                // Ansonsten liegt der Strahl innerhalb der Platte: t_min und t_max werden nicht beeinflusst.
            }
            else
            {
                // Berechne die Schnittpunkte t1 und t2 für die aktuelle Achse (X, Y oder Z)
                double t1 = (block.min_limits[i] - query_position[i]) / ray_direction[i];
                double t2 = (block.max_limits[i] - query_position[i]) / ray_direction[i];

                // Stelle sicher, dass t1 der kleinere Schnittpunkt ist
                if (t1 > t2) std::swap(t1, t2);
                
                // t_min ist der größte aller t1-Werte
                t_min = std::max(t_min, t1);
                // t_max ist der kleinste aller t2-Werte
                t_max = std::min(t_max, t2);
            }
        }

        // Wenn t_min > t_max, kreuzt der Strahl das AABB nicht.
        // Wenn t_max < 0, liegt das AABB hinter dem Strahl.
        if (t_min <= t_max && t_max >= 0.0)
        {
            // Der minimale positive Schnittpunkt t_hit ist t_min.
            // (Wenn t_min < 0, bedeutet das, dass query_position IM Block liegt, 
            // der Strahl tritt bei t_min < 0 ein und bei t_max > 0 aus. Wir verwenden t_max als Austrittspunkt.)
            
            double t_hit = t_min;
            if (t_min < 0.0) {
                // Der Punkt ist bereits innerhalb der Sicherheitszone des Blocks. 
                t_hit = 0.0; // Wir sind bereits in der Sicherheitszone
            }

            // Wir verfolgen den kleinsten positiven t-Wert.
            min_distance_t = std::min(min_distance_t, t_hit);
        }
    }
    
    // --- 2. Distanz zur äußeren Grenze (Workspace Limits - Safety Margin) ---
    
    // Führe eine weitere Ray-AABB-Schnittprüfung gegen den **erlaubten** Arbeitsraum durch.
    // Hier suchen wir den Schnittpunkt, an dem der Strahl den erlaubten Raum verlässt (t_max des erlaubten AABB).
    
    double workspace_t_min = std::numeric_limits<double>::lowest();
    double workspace_t_max = std::numeric_limits<double>::infinity();

    for (int i = 0; i < 3; ++i)
    {
        if (std::abs(ray_direction[i]) < 1e-6)
        {
            // Prüfen, ob der Startpunkt innerhalb der erlaubten AABB-Grenzen liegt
            if (query_position[i] < workspace_.min_limits[i] || query_position[i] > workspace_.max_limits[i])
            {
                // Wenn wir parallel zur Fläche laufen und außerhalb sind, 
                // haben wir den erlaubten Bereich bei t=0 verlassen.
                workspace_t_max = 0.0;
                break; // Beende die Schleife
            }
        }
        else
        {
            // Berechne die Schnittpunkte t1 und t2 für die aktuelle Achse
            double t1 = (workspace_.min_limits[i] - query_position[i]) / ray_direction[i];
            double t2 = (workspace_.max_limits[i] - query_position[i]) / ray_direction[i];

            if (t1 > t2) std::swap(t1, t2);

            // Wir sind an t_min interessiert (zum Eintreten) und t_max (zum Austreten)
            workspace_t_min = std::max(workspace_t_min, t1);
            workspace_t_max = std::min(workspace_t_max, t2);
        }
    }

    // Wenn der Strahl den erlaubten Bereich kreuzt und der Austrittspunkt positiv ist
    if (workspace_t_min <= workspace_t_max && workspace_t_max >= 0.0)
    {
        // Der Strahl verlässt den erlaubten Bereich bei workspace_t_max.
        // Wir sind nur am ersten Austritt interessiert, der vor uns liegt.
        double t_exit = workspace_t_max;
        
        // Wenn wir bereits außerhalb sind (query_position liegt außerhalb des effektiven AABB),
        // sollte der Strahl bereits bei t=0 den erlaubten Bereich verlassen haben.
        // Wir verwenden in diesem Fall t=0.
        if (workspace_t_min > 0.0) {
            // Strahl tritt erst nach t=0 in den erlaubten Bereich ein -> Der Punkt ist außerhalb.
            // Da wir die Distanz entlang der Geschwindigkeit suchen, muss dies der kürzeste Weg sein.
            // Der korrekteste Wert wäre die Distanz zur nächsten Grenze (t=0).
            t_exit = 0.0;
        }

        // Der kürzeste Abstand zum Verlassen des effektiven Arbeitsraums
        min_distance_t = std::min(min_distance_t, t_exit);
    }
    
    // Der Rückgabewert ist die minimale positive Distanz $t$
    return min_distance_t;
}


double SafetyLayer::getMaxSafeVelocity(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) 
{
    // The closest distance 's' is the remaining distance to the safety boundary.
    current_distance_to_obstacle = getShortestDistanceToSafetyBoundary(position);
    current_distance_to_obstacle_along_velocity_direction = getDistanceAlongVelocity(position, velocity);
   
    // safe_velocity = min(max_velocity, sqrt(2 * a * max(s, 0.0)))
    double safe_velocity = std::min(max_velocity_, min_velocity_ + std::sqrt(2 * safety_stopping_acceleration_ * std::max(current_distance_to_obstacle_along_velocity_direction, 0.0)));
    
    return safe_velocity;
}