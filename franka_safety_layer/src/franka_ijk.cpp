#include "../include/franka_safety_layer/franka_ijk.hpp"

Franka_IJK::Franka_IJK() : Node("franka_ijk") 
{

  // Setup Publisher and Subscriber
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("target_cartesian_pose", 1, std::bind(&Franka_IJK::targetPoseCallback, this, std::placeholders::_1));

  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&Franka_IJK::jointStateCallback, this, std::placeholders::_1));
        
  joint_velocity_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("franka_joint_trajectory_controller/joint_trajectory", 1);

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

  // Parameters for Initial Position
  this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));
  std::vector<double> init_joint_position_vec = this->get_parameter("init_joint_position").as_double_array();
  if (init_joint_position_vec.size() != 7 || std::all_of(init_joint_position_vec.begin(), init_joint_position_vec.end(), [](double x){ return std::abs(x) < 1e-6; }))
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid init joint positions (wrong size or all zeros). Shutting down.");
    rclcpp::shutdown();
    return;
  }
  q_init_ = Eigen::Map<Eigen::VectorXd>(init_joint_position_vec.data(), 7);
  RCLCPP_INFO(this->get_logger(), "Initial joint position loaded for nullspace control.");

  // Load Pinocchio Model
  if (!loadModel()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load Pinocchio model. Shutting down.");
    rclcpp::shutdown(); 
    return;
  }

  // 5. Setup Control Timer
  timer_ = this->create_wall_timer(std::chrono::duration<double>(TIME_STEP), std::bind(&Franka_IJK::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "franka_ijk initialized.");
}


bool Franka_IJK::loadModel()
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
  RCLCPP_INFO_ONCE(this->get_logger(), "Got target state");
  target_pose_stamped_ = *msg;
  target_se3_.translation() << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond q_rot(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
  target_se3_.rotation() = q_rot.toRotationMatrix();
}


bool Franka_IJK::getCurrentPose(pinocchio::SE3& current_se3)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      arm_prefix_ + "fr3_link0", arm_prefix_ + "fr3_link8", tf2::TimePointZero, std::chrono::milliseconds(100));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Could not transform: %s", ex.what());
    return false;
  }

  current_se3.translation() << transform_stamped.transform.translation.x,
                                transform_stamped.transform.translation.y,
                                transform_stamped.transform.translation.z;

  Eigen::Quaterniond q_rot(transform_stamped.transform.rotation.w,
                            transform_stamped.transform.rotation.x,
                            transform_stamped.transform.rotation.y,
                            transform_stamped.transform.rotation.z);
  current_se3.rotation() = q_rot.toRotationMatrix();

  return true;
}


void Franka_IJK::publishCommand(const Eigen::VectorXd& dq)
{
  // --- Publish Joint Command (trajectory_msgs::msg::JointTrajectory) ---
  auto traj_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
  
  // Set header
  traj_msg->header.stamp = rclcpp::Time(0, 0); //this->now();
  traj_msg->joint_names = {
      "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
      "fr3_joint5", "fr3_joint6", "fr3_joint7"
  };

  {
    // Create a single point for the velocity command
    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // Copy integrated position into the message point
    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.positions.push_back(q_(i) + dq(i) * TIME_STEP);
    }

    // Copy calculated velocities (dq) into the message point
    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.velocities.push_back(dq(i));
    }

    // Set duration from start (a small duration indicates this is an immediate command)
    point.time_from_start = rclcpp::Duration::from_seconds(TIME_STEP); 

    traj_msg->points.push_back(point);
  }

  {
    // Create a single point for the velocity command
    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // Copy integrated position into the message point
    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.positions.push_back(q_(i) + dq(i) * FINAL_TIME_STEP);
    }

    // Copy calculated velocities (dq) into the message point
    point.velocities.reserve(model_.nv);
    for (int i = 0; i < model_.nv; ++i) {
      point.velocities.push_back(dq(i));
    }

    // Set duration from start (a small duration indicates this is an immediate command)
    point.time_from_start = rclcpp::Duration::from_seconds(FINAL_TIME_STEP); 

    traj_msg->points.push_back(point);
  }

  joint_velocity_pub_->publish(std::move(traj_msg));
}


pinocchio::SE3 Franka_IJK::computeForwardKinematic(Eigen::VectorXd q)
{  

  // 1. Compute Forward Kinematics (FK)
  // Re-run the kinematics computation on the current joint position q_
  // The result is stored in the data_ structure.
  pinocchio::forwardKinematics(model_, *data_, q);
  
  // Update the frame placements (this is important if you use frames, not joints/bodies)
  pinocchio::updateFramePlacements(model_, *data_);

  // 2. Extract the pose of the end-effector frame
  // The frame pose is stored in data_->oMf[ee_frame_id_]
  // data_->oMf is the placement of the frame in the **world frame** (0)
  pinocchio::SE3 current_se3 = data_->oMf[ee_frame_id_]; 
  
  return current_se3;
}

/*
Eigen::VectorXd Franka_IJK::computeIKResult(const Eigen::VectorXd& desired_cartesian_velocity)
{

  Eigen::VectorXd q = pinocchio::neutral(model);
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-6;

  pinocchio::Data::Matrix6x J(6, model.nv);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);
  for (int i = 0;; i++)
  {
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
    err = pinocchio::log6(iMd).toVector(); // in joint frame
    if (err.norm() < eps)
    {
      success = true;
      break;
    }
    if (i >= IT_MAX)
    {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J); // J in joint frame
    pinocchio::Data::Matrix6 Jlog;
    pinocchio::Jlog6(iMd.inverse(), Jlog);
    J = -Jlog * J;
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
    if (!(i % 10))
      std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  if (success)
  {
    std::cout << "Convergence achieved!" << std::endl;
  }
  else
  {
    std::cout
      << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
      << std::endl;
  }

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err.transpose() << std::endl


  Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);

  const double IK_DT = 0.1;
  const double IK_TOLERANCE = 1e-4;
  const int IK_MAX_ITERS = 50;

  Eigen::VectorXd q_des = q_; // Initialize guess with current state

  bool success = pinocchio::computeInverseKinematics(
      model_, data_, target_se3_, ee_frame_id_, q_des,
      IK_DT, IK_TOLERANCE, IK_MAX_ITERS);

  if (success) {
      Eigen::VectorXd joint_error = q_des - q_;
      // P-controller in joint space: dq = K_Q * (q_des - q_curr)
      dq = K_Q * joint_error;
      RCLCPP_DEBUG(this->get_logger(), "IK Mode: Joint error norm: %.4f", joint_error.norm());
  } else {
      RCLCPP_WARN(this->get_logger(), "Inverse Kinematics failed to converge.");
  }
  

  return dq;
}
*/

Eigen::VectorXd Franka_IJK::computeCartesianVelocity(
  const pinocchio::SE3& current_se3, const pinocchio::SE3& target_se3)
{

  std::cout << "ccv: c" << std::endl;
  std::cout << current_se3 << std::endl;
  std::cout << "ccv: t" << std::endl;
  std::cout << target_se3 << std::endl;

  // Calculate the spatial error (difference) in the tangent space (R6)
  pinocchio::SE3 error = current_se3.inverse() * target_se3;
  Eigen::VectorXd cartesian_error = pinocchio::log6(error).toVector(); // R^6 error vector

  /*
  std::cout << "ccv: e" << std::endl;
  std::cout << error << std::endl;

  std::cout << "ccv: ce" << std::endl;
  std::cout << cartesian_error << std::endl;

  // P-controller for primary task: V_des = K_P * Error
  //Eigen::VectorXd desired_cartesian_velocity = K_P * cartesian_error; 
  */

  // 1. Calculate World Frame Position Error (for P-control only)
  Eigen::Vector3d p_curr = current_se3.translation();
  Eigen::Vector3d p_target = target_se3.translation();
  Eigen::Vector3d p_world_error = p_target - p_curr;

  // 2. Build the 6D desired velocity V_des (V_des = K_P * e)
  Eigen::VectorXd desired_cartesian_velocity = Eigen::VectorXd::Zero(6);

  // NOTE: The Jacobian (J) you use *must* be expressed in the World Frame (pinocchio::WORLD)
  // to be consistent with this World Frame error vector.
  desired_cartesian_velocity.head<3>() = K_PL * p_world_error;

  // --- 2. Compute Angular Error in World Frame (R3) ---
  
  // A. Calculate the relative rotation matrix R_err = R_current^-1 * R_target
  pinocchio::SE3 relative_pose = current_se3.inverse() * target_se3;
  Eigen::Matrix3d R_error = relative_pose.rotation();
  
  // B. Convert the relative rotation matrix R_err to an Axis-Angle vector (omega * theta).
  // This vector (omega * theta) is the magnitude and axis of the required rotation.
  // NOTE: pinocchio::log3(R_error) computes the 3D angular component (R3) of the Lie Logarithm map.
  // This vector is initially expressed in the CURRENT end-effector frame.
  Eigen::Vector3d angular_error_local = pinocchio::log3(R_error);

  // C. Rotate the Angular Error Vector from the Current End-Effector Frame to the World Frame.
  // To express a vector from the current frame into the world frame, multiply it by R_current.
  Eigen::Matrix3d R_current = current_se3.rotation();
  
  Eigen::Vector3d angular_error_world = R_current * angular_error_local;
  
  // Store in the bottom part of the 6D vector
  desired_cartesian_velocity.tail<3>() = K_PA * angular_error_world;

  // Limit velocity
  if (desired_cartesian_velocity.norm() > cartesian_velocity_limit_) {
    std::cout << "Cartesian velocity Limit" << std::endl;
    desired_cartesian_velocity *= cartesian_velocity_limit_ / desired_cartesian_velocity.norm();
  }

  // Apply tolerance to stop movement near target
  if (cartesian_error.norm() < 1e-3) {
    desired_cartesian_velocity.setZero();
  }

  std::cout << "ccv: cv" << std::endl;
  std::cout << desired_cartesian_velocity << std::endl;
  
  return desired_cartesian_velocity;
}



Eigen::VectorXd Franka_IJK::runJacobianNullspaceControl(const Eigen::VectorXd& desired_cartesian_velocity)
{

  // 1. Update Kinematics (needed for Jacobian calculation)
  pinocchio::computeAllTerms(model_, *data_, q_, Eigen::VectorXd::Zero(model_.nv));

  // 2. Compute the Jacobian matrix (6xN, N=DOF)
  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::getFrameJacobian(model_, *data_, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J);

  // 3. Compute the Damped Least Squares Pseudo-Inverse (J_dagger)
  const double lambda = 1e-6; // Damping factor for DLS
  Eigen::MatrixXd J_dagger = J.transpose() * (J * J.transpose() + lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();

  // 4. Primary Task: Cartesian Velocity
  // dq_prim = J_dagger * V_des
  Eigen::VectorXd dq_prim = J_dagger * desired_cartesian_velocity;

  // 5. Secondary Task: Nullspace Posture Control
  // 5.2. Compute the Nullspace Projector: N = I - J_dagger * J
  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model_.nv, model_.nv) - J_dagger * J;

  // 5.3. Secondary Task Velocity (Posture Control): dq_null_task = K_NULL * (q_init - q_curr)
  Eigen::VectorXd joint_error_posture = q_init_ - q_;
  Eigen::VectorXd dq_null_task = K_NULL * joint_error_posture;

  // 5.4. Projected Nullspace Command: dq_null = N * dq_null_task
  Eigen::VectorXd dq_null = N * dq_null_task;

  // 5.6. Combine Commands: dq = dq_prim + dq_null
  Eigen::VectorXd dq = dq_prim + dq_null;

  std::cout << "dq_prim" << std::endl;
  std::cout << dq_prim << std::endl;

  std::cout << "dq_null" << std::endl;
  std::cout << dq_null << std::endl;
  
  std::cout << "dq" << std::endl;
  std::cout << dq << std::endl;


  return dq;
}


// =================================================================================
// Control Loop (Main Orchestrator)
// =================================================================================

void Franka_IJK::controlLoop()
{
  std::cout << "Loop" << std::endl;

  if (target_pose_stamped_.header.stamp.sec == 0) {
    return;
  }

  if (q_.size() == 0 || q_.allFinite() == false) {
    return;
  }

  pinocchio::SE3 current_se3 = computeForwardKinematic(q_);

  // --- 2. Compute Primary Task (Cartesian Error and Velocity) ---
  Eigen::VectorXd desired_cartesian_velocity = computeCartesianVelocity(current_se3, target_se3_);
  
  Eigen::VectorXd dq;
  if (use_ik) {
    // --- 3a. Run IK Control (Position-based) ---
    RCLCPP_DEBUG(this->get_logger(), "Control Mode: IK (Position Tracking)");
    // Eigen::VectorXd target_q = computeIKResult(current_se3, target_se3);
    dq = Eigen::VectorXd::Zero(model_.nv);
    RCLCPP_DEBUG(this->get_logger(), "Not implemented");
  } 
  else 
  {
    // --- 3b. Run Jacobian + Nullspace Control (Velocity-based) ---
    // This mode executes the primary Cartesian task and the secondary Posture task.
    RCLCPP_DEBUG(this->get_logger(), "Control Mode: Jacobian + Nullspace (Velocity)");
    dq = runJacobianNullspaceControl(desired_cartesian_velocity);
  }
  // --- 4. Velocity Limiting 

  // Apply velocity limit
  double max_dq = dq.array().abs().maxCoeff();
  if (max_dq > joint_velocity_limit_) {
    RCLCPP_DEBUG(this->get_logger(), "Joint velocity Limit");
    dq *= (joint_velocity_limit_ / max_dq);
  }

  std::cout << "Cartesian Velocity" << std::endl << desired_cartesian_velocity << std::endl;
  std::cout << "Joint Velocity" << std::endl << dq << std::endl;

  // --- 5. Publish Command ---
  publishCommand(dq);

  std::cout << "cmd_se3" << std::endl;
  pinocchio::SE3 cmd_se3 = computeForwardKinematic(q_ + dq * 0.5);
  std::cout << cmd_se3 << std::endl;

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