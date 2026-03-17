#include "../include/franka_safety_layer/franka_ijk.hpp"

Franka_IJK::Franka_IJK() : Node("franka_ijk") 
{

  // Setup Publisher and Subscriber
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 1, std::bind(&Franka_IJK::targetPoseCallback, this, std::placeholders::_1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&Franka_IJK::jointStateCallback, this, std::placeholders::_1));
        
  joint_velocity_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("franka_joint_trajectory_controller/joint_trajectory", 1);
  debug_pub_ = this->create_publisher<franka_custom_msgs::msg::FIJKDebug>("fijk_debug", 1);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("safety_vis", 10);
  safe_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("safe_target_pose", 1);

  // TF Setup (for current pose access)
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters for arm prefix
  this->declare_parameter("arm_prefix", "franka_undefined");
  arm_prefix_ = this->get_parameter("arm_prefix").as_string();
  if(arm_prefix_ != "") 
  {
    arm_prefix_ += "_";
  }

  std::string other_ns;
  if(arm_prefix_ == "franka_left_") {
    other_ns = "franka_right";
  }
  else if(arm_prefix_ == "franka_right_") {
    other_ns = "franka_left";
  }
  else {
    other_ns = "franka_undefined";
  }
  safety_layer_.other_prefix_ = other_ns;

  std::cout << arm_prefix_ << " " << other_ns << std::endl;

  other_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/"+other_ns+"/joint_states", 1, std::bind(&Franka_IJK::otherJointStateCallback, this, std::placeholders::_1));

  // Parameters for Initial Position
  this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));
  std::vector<double> init_joint_position_vec = this->get_parameter("init_joint_position").as_double_array();

  this->declare_parameter("bypass_safety", false);
  bypass_safety_ = this->get_parameter("bypass_safety").as_bool();
  if (init_joint_position_vec.size() != 7 || std::all_of(init_joint_position_vec.begin(), init_joint_position_vec.end(), [](double x){ return std::abs(x) < 1e-6; }))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid init joint positions (wrong size or all zeros). Shutting down.");
    rclcpp::shutdown();
    return;
  }
  q_init_ = Eigen::Map<Eigen::VectorXd>(init_joint_position_vec.data(), 7);
  RCLCPP_INFO(this->get_logger(), "Initial joint position loaded for nullspace control.");

  this->declare_parameter("drift_correction_gain", 1.0);
  sync_gain_ = this->get_parameter("drift_correction_gain").as_double();
  
  this->declare_parameter("dq_filter_gain", 1.0);
  dq_filter_gain_ = this->get_parameter("dq_filter_gain").as_double();

  RCLCPP_INFO(this->get_logger(), "Virtual State initialized. K_SYNC: %f, K_FILTER: %f", sync_gain_, dq_filter_gain_);

  // Load Pinocchio Model
  if (!loadPinocchioModel()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load Pinocchio model. Shutting down.");
    rclcpp::shutdown(); 
    return;
  }

  // Load Pinocchio Model
  if(safety_layer_.other_robot_check && !bypass_safety_)
  {
    if (!loadOtherPinocchioModel(other_ns)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load other Pinocchio model. Shutting down.");
      rclcpp::shutdown(); 
      return;
    }
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Not loading other Pinocchio model.");
  }
  
  if (!bypass_safety_) {
    auto init_cartesian = computeForwardKinematic(q_init_).translation();
    safety_layer_.init(init_cartesian, marker_pub_);
  }

  RCLCPP_INFO(this->get_logger(), "franka_ijk initialized.");
}


bool Franka_IJK::loadPinocchioModel()
{
  // Create parameter client to get robot_description
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

  RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher parameter server...");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher to be available...");
  }

  // Wait until the parameter exists
  while (!param_client->has_parameter("robot_description")) {
    RCLCPP_INFO(this->get_logger(), "Waiting for 'robot_description' parameter...");
    rclcpp::sleep_for(500ms);
  }

  std::string urdf_string = param_client->get_parameter<std::string>("robot_description");
  if (urdf_string.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty robot_description from robot_state_publisher.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully fetched robot_description parameter from robot_state_publisher");

  // Load Pinocchio model from URDF string
  try {
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from parameter: %s", e.what());
    return false;
  }

  std::string end_effector_link_ = arm_prefix_ + "fr3_link8";

  if (!model_.existFrame(end_effector_link_)) {
    RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", end_effector_link_.c_str());
    return false;
  }

  ee_frame_id_ = model_.getFrameId(end_effector_link_);
  return true;
}


void Franka_IJK::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Got joint state");
  q_ = Eigen::Map<Eigen::VectorXd>(msg->position.data(), model_.nv);
}


void Franka_IJK::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  try {
    tf_buffer_->transform(*msg, target_pose_stamped_, target_frame_);
  } 
  catch (tf2::TransformException & ex) 
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Could not transform target pose from '%s' to '%s': %s",
                  msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    return; 
  }

  target_se3_.translation() << target_pose_stamped_.pose.position.x, target_pose_stamped_.pose.position.y, target_pose_stamped_.pose.position.z;

  Eigen::Quaterniond q_rot( target_pose_stamped_.pose.orientation.w, 
                            target_pose_stamped_.pose.orientation.x, 
                            target_pose_stamped_.pose.orientation.y, 
                            target_pose_stamped_.pose.orientation.z);
  target_se3_.rotation() = q_rot.toRotationMatrix();

  RCLCPP_INFO_ONCE(this->get_logger(), "Got and transformed target state");
  
  // run the control loop every time when target_cartesian_pose is published
  controlLoop();
}


// other robot
void Franka_IJK::otherJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Other joint cb");
  if (!bypass_safety_) {
    safety_layer_.other_q_ = Eigen::Map<Eigen::VectorXd>(msg->position.data(), 7);
  }
}


bool Franka_IJK::loadOtherPinocchioModel(std::string other_ns)
{
  // Create parameter client to get robot_description
  auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/"+other_ns+"/robot_state_publisher");

  RCLCPP_INFO(this->get_logger(), "Waiting for other robot_state_publisher parameter server...");
  while (!param_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for other robot_state_publisher to be available...");
  }

  // Wait until the parameter exists
  while (!param_client->has_parameter("robot_description")) {
    RCLCPP_INFO(this->get_logger(), "Waiting for other 'robot_description' parameter...");
    rclcpp::sleep_for(500ms);
  }

  std::string urdf_string = param_client->get_parameter<std::string>("robot_description");
  if (urdf_string.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received empty robot_description from other robot_state_publisher.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully fetched other robot_description parameter from other robot_state_publisher");

  // Load Pinocchio model from URDF string
  try {
    pinocchio::urdf::buildModelFromXML(urdf_string, safety_layer_.other_model_);
    safety_layer_.other_data_ = std::make_unique<pinocchio::Data>(safety_layer_.other_model_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from parameter: %s", e.what());
    return false;
  }

  std::string end_effector_link_ = other_ns + "_fr3_link8";

  if (!safety_layer_.other_model_.existFrame(end_effector_link_)) {
    RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", end_effector_link_.c_str());
    return false;
  }

  safety_layer_.other_ee_frame_id_ = safety_layer_.other_model_.getFrameId(end_effector_link_);
  return true;
}


bool Franka_IJK::tfLookup(std::string frame_from, std::string frame_to, pinocchio::SE3 &result)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      frame_from, frame_to, tf2::TimePointZero, std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Could not transform: %s", ex.what());
    return false;
  }

  result.translation() << transform_stamped.transform.translation.x,
                                transform_stamped.transform.translation.y,
                                transform_stamped.transform.translation.z;

  Eigen::Quaterniond q_rot(transform_stamped.transform.rotation.w,
                            transform_stamped.transform.rotation.x,
                            transform_stamped.transform.rotation.y,
                            transform_stamped.transform.rotation.z);
  result.rotation() = q_rot.toRotationMatrix();

  return true;
}


geometry_msgs::msg::Pose Franka_IJK::convert(pinocchio::SE3 se3)
{
  geometry_msgs::msg::Pose pose_msg;

  pose_msg.position.x = se3.translation()(0);
  pose_msg.position.y = se3.translation()(1);
  pose_msg.position.z = se3.translation()(2);

  Eigen::Quaterniond q(se3.rotation());
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();

  return pose_msg;
}


geometry_msgs::msg::Twist Franka_IJK::convert(Eigen::VectorXd v)
{
  geometry_msgs::msg::Twist twist_msg;

  if (v.size() != 6) {
    return twist_msg;
  }

  twist_msg.linear.x = v(0);
  twist_msg.linear.y = v(1);
  twist_msg.linear.z = v(2);

  twist_msg.angular.x = v(3);
  twist_msg.angular.y = v(4);
  twist_msg.angular.z = v(5);

  return twist_msg;
}


pinocchio::SE3 Franka_IJK::computeForwardKinematic(Eigen::VectorXd q)
{  
  pinocchio::forwardKinematics(model_, *data_, q);
  pinocchio::updateFramePlacements(model_, *data_);
  return data_->oMf[ee_frame_id_]; 
}


double Franka_IJK::computeCartesianVelocity(
  const pinocchio::SE3 &current_se3, const pinocchio::SE3 &target_se3, Eigen::VectorXd &desired_cartesian_velocity)
{

  // Linear: Compute Error in World Frame
  Eigen::Vector3d lin_world_error = target_se3.translation() - current_se3.translation();
  
  // Angular 
  // 1. Calculate the relative rotation matrix R_err = R_current^-1 * R_target
  pinocchio::SE3 relative_pose = current_se3.inverse() * target_se3;
  Eigen::Matrix3d R_error = relative_pose.rotation();
  Eigen::Vector3d angular_error_local = pinocchio::log3(R_error);

  // 2. Rotate the Angular Error Vector from the Current End-Effector Frame to the World Frame.
  Eigen::Matrix3d R_current = current_se3.rotation();
  Eigen::Vector3d angular_world_error = R_current * angular_error_local;
  
  // Build the 6D desired velocity V = K * e
  desired_cartesian_velocity = Eigen::VectorXd::Zero(6);
  desired_cartesian_velocity.head<3>() = lin_world_error / MOTION_TIME_STEP;
  desired_cartesian_velocity.tail<3>() = angular_world_error / MOTION_TIME_STEP;

  double target_reachable_factor = 1;

  // Limit velocity
  if (desired_cartesian_velocity.norm() > cartesian_velocity_limit_) 
  {
    double reduction = cartesian_velocity_limit_ / desired_cartesian_velocity.norm();
    target_reachable_factor = 1 / reduction;
    desired_cartesian_velocity *= reduction;
  }

  if (!bypass_safety_)
  {
    double max_safe_v = safety_layer_.getMaxSafeVelocity(current_se3.translation(), desired_cartesian_velocity);
    if (desired_cartesian_velocity.norm() > max_safe_v)
    {
      double reduction = max_safe_v / desired_cartesian_velocity.norm();
      target_reachable_factor = 1;
      desired_cartesian_velocity *= reduction;
    } 
  }

  // Apply tolerance to stop movement near target
  if (desired_cartesian_velocity.norm() < 1e-3) {
    desired_cartesian_velocity.setZero();
  }
  
  return target_reachable_factor;
}


Eigen::VectorXd Franka_IJK::runJacobianNullspaceControl(const Eigen::VectorXd& desired_cartesian_velocity)
{

  // 1. Update Kinematics (needed for Jacobian calculation)
  pinocchio::computeAllTerms(model_, *data_, q_v_, Eigen::VectorXd::Zero(model_.nv));

  // 2. Compute the Jacobian matrix (6xN, N=DOF)
  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // 3. Compute the Damped Least Squares Pseudo-Inverse (J_dagger)
  const double lambda = 1e-6; // Damping factor for DLS
  Eigen::MatrixXd J_dagger = J.transpose() * (J * J.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();

  // 4. Primary Task: Cartesian Velocity
  Eigen::VectorXd dq_prim = J_dagger * desired_cartesian_velocity;

  // 5. Secondary Task: Nullspace Posture Control
  // 5.2. Compute the Nullspace Projector: N = I - J_dagger * J
  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model_.nv, model_.nv) - J_dagger * J;

  // 5.3. Secondary Task Velocity (Posture Control): dq_null_task = K_NULL * (q_init - q_curr)
  Eigen::VectorXd joint_error_posture = q_init_ - q_v_;
  Eigen::VectorXd dq_null_task = K_NULL * joint_error_posture;

  // 5.4. Projected Nullspace Command: dq_null = N * dq_null_task
  Eigen::VectorXd dq_null = N * dq_null_task;

  // 5.6. Combine Commands: dq = dq_prim + dq_null
  Eigen::VectorXd dq = dq_prim + dq_null;

  return dq;
}

void Franka_IJK::controlLoop()
{

  if (q_.size() == 0 || q_.allFinite() == false) {
    return;
  }

  // Initialize Virtual State if needed
  if (!initialized_v_) {
    if (q_.size() == model_.nv) {
      q_v_ = q_;
      dq_filtered_ = Eigen::VectorXd::Zero(model_.nv);
      initialized_v_ = true;
      RCLCPP_INFO(this->get_logger(), "Internal Virtual State Initialized.");
    } else {
      return; 
    }
  }

  // --- Drift Correction ---
  // Gently pull virtual state to real state (Leaky Integrator on Position)
  if (sync_gain_ > 0.0) {
      q_v_ += sync_gain_ * (q_ - q_v_);
  }

  // --- 1. Update Kinematics and Dynamics (Gravity) ---
  // Ensure gravity vector data_->g is updated every cycle for dynamic effort
  // We use q_v_ for consistency, so gravity compensation is smooth
  pinocchio::computeGeneralizedGravity(model_, *data_, q_v_);


  if (target_pose_stamped_.header.stamp.sec == 0) 
  {
    pinocchio::SE3 current_se3 = computeForwardKinematic(q_v_);
    Eigen::VectorXd desired_cartesian_velocity;
    computeCartesianVelocity(current_se3, current_se3, desired_cartesian_velocity);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);
    publishDebugInfos(current_se3, current_se3, current_se3, desired_cartesian_velocity, dq);
    if (!bypass_safety_) {
        safety_layer_.vis_.publish_markers();
    }
    return;
  }

  //pinocchio::SE3 tf_se3;
  //if(!tfLookup(arm_prefix_ + "fr3_link0", arm_prefix_ + "fr3_link8", tf_se3)) {
  //  return;
  //}
  
  pinocchio::SE3 current_se3 = computeForwardKinematic(q_v_);

  pinocchio::SE3 safe_target_se3;
  if (bypass_safety_)
  {
      safe_target_se3 = target_se3_;
  }
  else
  {
      safe_target_se3 = safety_layer_.adjustToSafePose(current_se3, target_se3_);
  }

  // --- 2. Compute Primary Task (Cartesian Error and Velocity) ---
  Eigen::VectorXd desired_cartesian_velocity;
  double target_reachable_factor = computeCartesianVelocity(current_se3, safe_target_se3, desired_cartesian_velocity);
  
  Eigen::VectorXd dq;
  if (use_ik) {
    // --- 3a. Run IK Control (Position-based) ---
    // Eigen::VectorXd target_q = computeIKResult(current_se3, target_se3);
    dq = Eigen::VectorXd::Zero(model_.nv);
    RCLCPP_FATAL(this->get_logger(), "Not implemented");
  } 
  else 
  {
    // This mode executes the primary Cartesian task and the secondary Posture task.
    dq = runJacobianNullspaceControl(desired_cartesian_velocity);
  }

  // --- 3c. Output Velocity Filtering ---
  if (first_run_) {
    dq_filtered_ = dq;
    first_run_ = false;
  }
  else {
    // Simple Low-Pass Filter on Velocity
    dq_filtered_ += (dq - dq_filtered_) * dq_filter_gain_;
  }
  dq = dq_filtered_;

  // --- 4. Integrate Virtual State ---
  // This is the core of the "Virtual Robot" approach.
  // We drive q_v_ with the filtered smooth velocity.
  q_v_ += dq * TIME_STEP;
  // --- 4. Velocity Limiting 
  double max_dq = dq.array().abs().maxCoeff();
  if (max_dq > joint_velocity_limit_) {
    RCLCPP_DEBUG(this->get_logger(), "Joint velocity Limit");
    dq *= (joint_velocity_limit_ / max_dq);
  }

  // --- 5. Publish Command ---
  publishCommand(dq);

  // --- 6. Publish safety-clamped target pose for cartesian_impedance_controller ---
  {
    geometry_msgs::msg::PoseStamped safe_pose_msg;
    safe_pose_msg.header.stamp = this->get_clock()->now();
    safe_pose_msg.header.frame_id = target_frame_;
    safe_pose_msg.pose = convert(safe_target_se3);
    safe_pose_pub_->publish(safe_pose_msg);
  }

  publishDebugInfos(current_se3, target_se3_, safe_target_se3, desired_cartesian_velocity, dq);

  if (!bypass_safety_) {
      safety_layer_.vis_.publish_markers();
  }
}


void Franka_IJK::publishCommand(const Eigen::VectorXd& dq)
{
  auto traj_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  
  traj_msg->header.stamp = this->get_clock()->now(); // = rclcpp::Time(0, 0);
  traj_msg->joint_names = {
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7"
  };

  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(MOTION_TIME_STEP); 
    
    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.positions.push_back(q_(i) + dq(i) * MOTION_TIME_STEP);
    }

    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.velocities.push_back(dq(i));
    }    
    
    // point.accelerations.assign(model_.nv, 0);

    traj_msg->points.push_back(point);
  }

  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(FINAL_TIME_STEP); 
    
    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.positions.push_back(q_(i) + dq(i) * FINAL_TIME_STEP);
    }
    
    point.velocities.assign(model_.nv, 0);
    
    // point.accelerations.assign(model_.nv, 0);

    traj_msg->points.push_back(point);
  }

  joint_velocity_pub_->publish(std::move(traj_msg));
}


void Franka_IJK::publishDebugInfos(pinocchio::SE3 &current_se3, pinocchio::SE3 &target_se3, pinocchio::SE3 &safe_target_se3, Eigen::VectorXd &desired_cartesian_velocity, Eigen::VectorXd &dq)
{
  franka_custom_msgs::msg::FIJKDebug debug_msg;
  debug_msg.header.stamp = this->get_clock()->now();
  debug_msg.header.frame_id = target_frame_;

  debug_msg.actual_pose = convert(current_se3);
  debug_msg.target_pose = convert(target_se3);
  debug_msg.safe_target_pose = convert(safe_target_se3);
  debug_msg.cmd_pose = convert(computeForwardKinematic(q_v_ + dq * MOTION_TIME_STEP));
  debug_msg.cmd_final_pose = convert(computeForwardKinematic(q_v_ + dq * FINAL_TIME_STEP));
  debug_msg.cartesian_velocity = convert(desired_cartesian_velocity);

  debug_msg.safety_distance = safety_layer_.current_distance_to_obstacle;
  debug_msg.safety_distance_along_velocity = safety_layer_.current_distance_to_obstacle_along_velocity_direction;

  debug_pub_->publish(std::move(debug_msg));
}




// =================================================================================
// Main Function
// =================================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting Cartesian Motion Controller Node...");
  auto node = std::make_shared<Franka_IJK>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}