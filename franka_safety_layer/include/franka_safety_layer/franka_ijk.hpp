#pragma once

#include <memory>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>

// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include "franka_custom_msgs/msg/fijk_debug.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
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

#include "safety_layer.hpp"

using namespace std::chrono_literals;

class Franka_IJK : public rclcpp::Node
{
public:
  
  Franka_IJK();

private:

  bool loadPinocchioModel();
  bool loadOtherPinocchioModel(std::string other_ns);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void otherJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void otherRobotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg);
  
  bool tfLookup(std::string frame_from, std::string frame_to, pinocchio::SE3 &result);

  geometry_msgs::msg::Pose convert(pinocchio::SE3 se3);
  geometry_msgs::msg::Twist convert(Eigen::VectorXd v);

  pinocchio::SE3 computeForwardKinematic(Eigen::VectorXd q);
  double computeCartesianVelocity(const pinocchio::SE3& current_se3, const pinocchio::SE3& target_se3, Eigen::VectorXd &desired_cartesian_velocity);
  Eigen::VectorXd runJacobianNullspaceControl(const Eigen::VectorXd& desired_cartesian_velocity);
  void controlLoop();

  void publishCommand(double target_reachable_factor, const Eigen::VectorXd& dq);
  void publishDebugInfos(pinocchio::SE3 &current_se3, pinocchio::SE3 &target_se3, pinocchio::SE3 &safe_target_se3, Eigen::VectorXd &desired_cartesian_velocity, Eigen::VectorXd &dq);

  // ROS 2 components
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_, other_joint_state_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr other_robot_description_subscriber_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_velocity_pub_; 
  rclcpp::Publisher<franka_custom_msgs::msg::FIJKDebug>::SharedPtr debug_pub_; 
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF components
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // sub classes
  SafetyLayer safety_layer_;

  std::string arm_prefix_;

  // Pinocchio model and data
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex ee_frame_id_;

  // State variables
  std::string target_frame_ = "base";
  geometry_msgs::msg::PoseStamped target_pose_stamped_;
  pinocchio::SE3 target_se3_; // Target pose (Pinocchio format)

  Eigen::VectorXd q_;          // Current joint configuration (assumed/integrated state)
  Eigen::VectorXd q_init_;     // Initial joint configuration for nullspace posture control

  bool use_ik = false;
  double joint_velocity_limit_ = 0.5; // Max joint velocity in rad/s
  double cartesian_velocity_limit_ = 0.5; // Max joint velocity in m/s

  // Control gains
  const double K_NULL = 1.0;  // Gain for nullspace posture task (Secondary Task)

  const double frequency = 15;
  const double TIME_STEP = 1.0 / frequency; // Control loop frequency (15 Hz)
  const double MOTION_TIME_STEP = 1.0 * TIME_STEP; 
  const double FINAL_TIME_STEP = 5 * TIME_STEP;

};
