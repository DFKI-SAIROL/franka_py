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

#include <franka_adaptable_controllers/joint_position_adaptable_controller.hpp>
#include <franka_adaptable_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_adaptable_controllers {

controller_interface::InterfaceConfiguration JointPositionAdaptableController::command_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for (int i = 1; i <= num_joints; ++i) 
  {
    if(control_joint_position_)
    {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
    else
    {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration JointPositionAdaptableController::state_interface_configuration() const 
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) 
  {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  // add the robot time interface
  if (!is_gazebo_) 
  {
    config.names.push_back(arm_id_ + "/robot_time");
  }

  return config;
}

controller_interface::return_type JointPositionAdaptableController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) 
{
  if (initialization_flag_) 
  {

    initialization_flag_ = false;
    elapsed_time_ = 0.0;
    
    for (int i = 0; i < num_joints; ++i) 
    {
      initial_q_[i] = state_interfaces_[2*i].get_value();
      motion_goal_position_[i] = initial_q_[i];
      RCLCPP_WARN(get_node()->get_logger(), "JOAC: init q %d %.4f", i, initial_q_[i]);
    }

    if (!is_gazebo_) 
    {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
  } 
  else 
  {
    if (!is_gazebo_) 
    {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } 
    else 
    {
      elapsed_time_ += trajectory_period_;
    }
  }

  sensor_msgs::msg::JointState debug_msg;
  for (int i = 0; i < num_joints; ++i) 
  {  
    debug_msg.name.push_back("joint_" + std::to_string(i));
  }

  if(use_target_directly_)
  {
    if(!control_joint_position_) RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), log_throttle_duration, "Not implemented");

    for (int i = 0; i < num_joints; ++i) 
    {    
      if (i==6) RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), log_throttle_duration, "JOAC: target %d: %f", i, motion_goal_position_[i]);
      if(control_joint_position_)
      {
        command_interfaces_[i].set_value(motion_goal_position_[i]);    
      }
      else
      {
        command_interfaces_[i].set_value(0);    
      }
    }
  }
  else
  {
    if(is_in_motion_ && (elapsed_time_ > motion_start_time_ + motion_duration_ || motion_duration_ == 0))
    {
      double max_joint_distance = 0;
      for (int i = 0; i < num_joints; ++i) 
      {
        max_joint_distance = std::max(max_joint_distance, std::abs(motion_goal_position_[i] - state_interfaces_[2*i].get_value()));
      }
      RCLCPP_INFO(get_node()->get_logger(), "motion error: %f", max_joint_distance);
      if(max_joint_distance > restart_joint_distance_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Restart motion");
        computeMotion(motion_goal_position_);
      }
      else
      {
        is_in_motion_ = false;
      }
    }

    if(!is_in_motion_)
    {
      for (int i = 0; i < num_joints; ++i) 
      {
        double actual_q = state_interfaces_[2*i].get_value();
        double actual_dq = state_interfaces_[2*i+1].get_value();

        if(control_joint_position_)
        {
          command_interfaces_[i].set_value(motion_goal_position_[i]);    
        }
        else
        {
          command_interfaces_[i].set_value(0);    
        }

        if (i==6) RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), log_throttle_duration, "JOAC %d: actual %f %f, motion %f, 0", i, actual_q, actual_dq, motion_goal_position_[i]);
        debug_msg.position.push_back(motion_goal_position_[i]); 
        debug_msg.velocity.push_back(0); 
        debug_msg.effort.push_back(0); 
      }
    }
    else 
    {
      for (int i = 0; i < num_joints; ++i) 
      {    
        double tau = (elapsed_time_ - motion_start_time_) / motion_duration_;
        double h_tau = 10*std::pow(tau,3) - 15*std::pow(tau,4) + 6*std::pow(tau,5);
        double hd_tau = 30*std::pow(tau,2) - 60*std::pow(tau,3) + 30*std::pow(tau,4);
        double hdd_tau = 60*std::pow(tau,1) - 180*std::pow(tau,2) + 120*std::pow(tau,3);
        double current_q = motion_start_position_[i] + (motion_goal_position_[i] - motion_start_position_[i]) * h_tau;
        double current_dq = (1 / motion_duration_) * (motion_goal_position_[i] - motion_start_position_[i]) * hd_tau;
        double current_ddq = (1 / std::pow(motion_duration_, 2)) * (motion_goal_position_[i] - motion_start_position_[i]) * hdd_tau;
        double actual_q = state_interfaces_[2*i].get_value();
        double actual_dq = state_interfaces_[2*i+1].get_value();

        if(control_joint_position_)
        {
          command_interfaces_[i].set_value(current_q);    
        }
        else
        {
          command_interfaces_[i].set_value(current_dq);    
        }

        if (i==6) RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), log_throttle_duration, "JOAC %d: actual %f %f, motion %f %f %f", i, actual_q, actual_dq, current_q, current_dq, current_ddq);
        debug_msg.position.push_back(current_q); 
        debug_msg.velocity.push_back(current_dq); 
        debug_msg.effort.push_back(current_ddq); 
      }
    }
  }

  debug_publisher_->publish(debug_msg);

  return controller_interface::return_type::OK;
}

double JointPositionAdaptableController::calculateT(double delta_q, double max_joint_velocity, double max_joint_acceleration)
{
  double Tv = (15.0/8.0) * (std::abs(delta_q) / (motion_duration_safety_factor_ * max_joint_velocity));
  double Ta = std::sqrt((10.0 * std::sqrt(3.0) / 3.0) * (abs(delta_q) / (motion_duration_safety_factor_ * max_joint_acceleration)));
  double T = std::max(Tv, Ta);
  return T;
}

void JointPositionAdaptableController::targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{

  if(!use_target_directly_ && is_in_motion_) 
  {
    RCLCPP_WARN(get_node()->get_logger(), "Received new target while in motion. Ignoring");
    return;
  }

  if (int(msg->data.size()) != num_joints) 
  {
    RCLCPP_WARN(get_node()->get_logger(), "Received target_q with %zu elements, expected %d. Ignoring.", msg->data.size(), num_joints);
    return;
  }

  std::array<double, 7> last_motion_goal_position;
  for (int i = 0; i < num_joints; ++i) 
  {
    last_motion_goal_position[i] = motion_goal_position_[i];
    if(is_target_relative_)
    {
      motion_goal_position_[i] = initial_q_[i] + msg->data[i];
    }
    else
    {
      motion_goal_position_[i] = msg->data[i];
    }
  }

  if(!use_target_directly_)
  {
    computeMotion(last_motion_goal_position);
  }

  RCLCPP_INFO(get_node()->get_logger(), "Updated target_q from topic, start new motion with duration %f", motion_duration_);
  for (int i = 0; i < num_joints; ++i) 
  {
    if (i==6) RCLCPP_INFO(get_node()->get_logger(), "JOAC cb %i: actual %f, target %f + %f -> %f", i, motion_start_position_[i], initial_q_[i], msg->data[i], motion_goal_position_[i]);
  }


}


void JointPositionAdaptableController::computeMotion(std::array<double, 7> &last_motion_goal_position_)
{
  
  is_in_motion_ = true;
  motion_start_time_ = elapsed_time_;
  
  // compute motion duration that respects joint limits
  motion_duration_ = 0;
  for (int i = 0; i < num_joints; ++i) 
  {
    if(control_joint_position_)
    {
      motion_start_position_[i] = last_motion_goal_position_[i];
    }
    else
    {
      motion_start_position_[i] = state_interfaces_[2*i].get_value();
    }
    double delta_q = motion_goal_position_[i] - motion_start_position_[i];
    motion_duration_ = std::max(motion_duration_, calculateT(delta_q, joint_velocity_limit_[i], joint_acceleration_limit_[i]));
  }

}

CallbackReturn JointPositionAdaptableController::on_init() 
{
  try 
  {
    auto_declare<bool>("gazebo", false);
    auto_declare<bool>("is_target_relative", true);
    auto_declare<bool>("use_target_directly", false);
    auto_declare<bool>("control_joint_position", false);
    auto_declare<std::string>("robot_description", "");
  } 
  catch (const std::exception& e) 
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionAdaptableController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();
  is_target_relative_ = get_node()->get_parameter("is_target_relative").as_bool();
  use_target_directly_ = get_node()->get_parameter("use_target_directly").as_bool();
  control_joint_position_ = get_node()->get_parameter("control_joint_position").as_bool();

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) 
  {
    robot_description_ = result[0].value_to_string();
  } 
  else 
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());
  bool succ = robot_utils::getJointVelocityLimitsFromDescription(robot_description_, get_node()->get_logger(), joint_velocity_limit_);
  if(!succ)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  target_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/target_joint_position", 1,
    std::bind(&JointPositionAdaptableController::targetCallback, this, std::placeholders::_1));

  debug_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>("~/debug", 1);
      
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionAdaptableController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) 
{
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_adaptable_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_adaptable_controllers::JointPositionAdaptableController,
                       controller_interface::ControllerInterface)
