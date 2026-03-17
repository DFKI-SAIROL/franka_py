// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_state.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_adaptable_controllers {

/**
 * The joint position adaptable controller moves in a periodic movement.
 */
class JointPositionAdaptableController : public controller_interface::ControllerInterface 
{
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  void computeMotion(std::array<double, 7> &last_motion_goal_position_);
  double calculateT(double delta_q, double max_joint_velocity, double max_joint_acceleration);
  void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

 private:
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_publisher_;

  std::string arm_id_;
  std::string arm_prefix_;
  std::string robot_description_;
  bool is_gazebo_ = false;
  bool use_fake_hardware_ = false;
  bool is_target_relative_ = true;
  bool use_target_directly_ = false;
  bool control_joint_position_ = false;

  const int num_joints = 7;
  double trajectory_period_ = 0.001;
  std::array<double, 7> joint_velocity_limit_{1, 1, 1, 1, 1, 1, 1}; // will be overwritten by robot_decription
  std::array<double, 7> joint_acceleration_limit_{10, 10, 10, 10, 10, 10, 1};

  bool initialization_flag_ = true;
  double initial_robot_time_ = 0.0;
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
  double elapsed_time_ = 0.0;
  double robot_time_ = 0.0;

  // motion
  bool is_in_motion_ = false;
  double motion_start_time_;
  double motion_duration_ = 0;
  double motion_duration_safety_factor_ = 0.2;
  double restart_joint_distance_ = 0.001;
  std::array<double, 7> motion_start_position_, motion_goal_position_;

  double log_throttle_duration = 100;


};

}  // namespace franka_adaptable_controllers
