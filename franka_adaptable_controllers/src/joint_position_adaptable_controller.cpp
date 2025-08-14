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
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
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
      initial_q_.at(i) = state_interfaces_[i].get_value();
    }

    if(is_target_relative_)
    {
      for (int i = 0; i < num_joints; ++i) 
      { 
        current_target_q_.at(i) = 0;
      }
    }
    else
    {
      for (int i = 0; i < num_joints; ++i) 
      { 
        current_target_q_.at(i) = state_interfaces_[i].get_value();
      }
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

  if(is_target_relative_)
  {
    for (int i = 0; i < num_joints; ++i) 
    {
      command_interfaces_[i].set_value(initial_q_.at(i) + current_target_q_.at(i));    
    }
  }
  else
  {
    for (int i = 0; i < num_joints; ++i) 
    {
      command_interfaces_[i].set_value(current_target_q_.at(i));    
    }
  }

  return controller_interface::return_type::OK;
}

void JointPositionAdaptableController::targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
  if (int(msg->data.size()) != num_joints) 
  {
    RCLCPP_WARN(get_node()->get_logger(), "Received target_q with %zu elements, expected %d. Ignoring.", msg->data.size(), num_joints);
    return;
  }

  for (int i = 0; i < num_joints; ++i) 
  {
    current_target_q_[i] = msg->data[i];
  }

  RCLCPP_DEBUG(get_node()->get_logger(), "Updated target_q from topic.");
}

CallbackReturn JointPositionAdaptableController::on_init() 
{
  try 
  {
    auto_declare<bool>("gazebo", false);
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
  is_target_relative_ = get_node()->get_parameter("target_relative").as_bool();

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

  target_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/target_joint_position", 1,
    std::bind(&JointPositionAdaptableController::targetCallback, this, std::placeholders::_1));

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
