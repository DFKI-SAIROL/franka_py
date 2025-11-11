#pragma once

#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Pinocchio Includes
#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>

using namespace std::chrono_literals;

class Franka_IJK : public rclcpp::Node
{
public:
  
  Franka_IJK();

private:

  bool loadModel();
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool getCurrentPose(pinocchio::SE3& current_se3);
  pinocchio::SE3 computeForwardKinematic(Eigen::VectorXd q);
  void controlLoop(); // Main orchestrator method
  Eigen::VectorXd computeCartesianVelocity(const pinocchio::SE3& current_se3, const pinocchio::SE3& target_se3);
  Eigen::VectorXd runIKControl(const Eigen::VectorXd& desired_cartesian_velocity);
  Eigen::VectorXd computeIKResult(const Eigen::VectorXd& desired_cartesian_velocity);
  Eigen::VectorXd runJacobianNullspaceControl(const Eigen::VectorXd& desired_cartesian_velocity);
  Eigen::VectorXd computeNullspaceDq(const Eigen::MatrixXd& J_dagger);
  void publishCommand(const Eigen::VectorXd& dq);

  // ROS 2 components
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_velocity_pub_; 
  rclcpp::TimerBase::SharedPtr timer_;

  // TF components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string arm_prefix_;

  // Pinocchio model and data
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex ee_frame_id_;

  // State variables
  geometry_msgs::msg::PoseStamped target_pose_stamped_;
  pinocchio::SE3 target_se3_; // Target pose (Pinocchio format)

  Eigen::VectorXd q_;          // Current joint configuration (assumed/integrated state)
  Eigen::VectorXd q_init_;     // Initial joint configuration for nullspace posture control

  bool use_ik = false;
  double joint_velocity_limit_ = 0.5; // Max joint velocity in rad/s
  double cartesian_velocity_limit_ = 0.5; // Max joint velocity in m/s

  // Control gains
  const double K_PL = 1.0;     // Proportional gain for Cartesian linear velocity control
  const double K_PA = 1.0;     // Proportional gain for Cartesian angular velocity control
  const double K_Q = 1.0;    // Proportional gain for joint position control (IK approach)
  const double K_NULL = 0.1;  // Gain for nullspace posture task (Secondary Task)
  const double TIME_STEP = 0.02; // Control loop frequency (50 Hz)
  const double FINAL_TIME_STEP = 0.1; // Control loop frequency (50 Hz)

};
