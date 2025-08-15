// Copyright (c) 2024 Franka Robotics GmbH
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

#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <tinyxml2.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_utils {

using namespace std::chrono_literals;
const auto time_out = 1000ms;  // This is probably not really necessary

struct JointControlInterfaces {
  std::vector<std::string> command_interfaces;  ///< e.g., {"position", "velocity", "effort"}
  std::vector<std::string> state_interfaces;    ///< e.g., {"position", "velocity", "effort"}
};

inline std::string getRobotNameFromDescription(const std::string& robot_description,
                                               const rclcpp::Logger& logger) {
  std::string robot_name{};
  tinyxml2::XMLDocument doc;

  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to parse robot_description");
    return robot_name;
  }

  const tinyxml2::XMLElement* robot_elem = doc.FirstChildElement("robot");
  if (!robot_elem) {
    RCLCPP_ERROR(logger, "No <robot> element found in robot_description");
    return robot_name;
  }

  const char* name_attr = robot_elem->Attribute("name");
  if (!name_attr || std::string(name_attr).empty()) {
    RCLCPP_ERROR(logger, "Robot name attribute missing or empty in <robot> element");
    return robot_name;
  }

  robot_name = name_attr;
  RCLCPP_INFO(logger, "Extracted robot name: %s", robot_name.c_str());
  return robot_name;
}


inline bool getJointVelocityLimitsFromDescription(const std::string& robot_description,
                                               const rclcpp::Logger& logger, std::array<double, 7> &joint_velocity_limits) {

  tinyxml2::XMLDocument doc;

  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to parse robot_description");
    return false;
  }

  tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
  if (!robot) {
    RCLCPP_ERROR(logger, "No <robot> element found in URDF");
    return false;
  }

  int i = 0;

  for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint")) {
    std::string joint_name = std::string(joint->Attribute("name"));
    std::string joint_type = std::string(joint->Attribute("type"));

    // Only consider movable joints (revolute, continuous, prismatic)
    if (joint_type != "revolute" && joint_type != "continuous" && joint_type != "prismatic") {
      continue;
    }

    tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
    if (!limit) {
      continue;
    }

    double velocity = 0.0;
    if (limit->QueryDoubleAttribute("velocity", &velocity) != tinyxml2::XML_SUCCESS) {
      continue; // no velocity attribute
    }

    if(i >= 7) {
      RCLCPP_ERROR(logger, "Too many limits %s %s %f", joint_name.c_str(), joint_type.c_str(), velocity);
      continue;
    }

    int joint_number = joint_name.back() - '0';  // char → int
    if(joint_number != i+1){
      RCLCPP_ERROR(logger, "Joint name number does not match array index %d %d", joint_number, i);
    }

    RCLCPP_INFO(logger, "Limit %s %d %f", joint_name.c_str(), joint_number, velocity);
    joint_velocity_limits[i] = velocity;
    i++;
  }

  return true;
}


// Extracts the joint names and their corresponding command and state interfaces
// from robot_description XML string.
// Populates the joint_interfaces map with
// key: joint name
// value: a JointControlInterfaces structure containing all configured
//        command_interface and state_interface names
inline bool getJointControlInterfaces(
    const std::string& robot_description,
    const rclcpp::Logger& logger,
    std::unordered_map<std::string, JointControlInterfaces>& joint_interfaces) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(robot_description.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to parse robot_description");
    return false;
  }

  const tinyxml2::XMLElement* robot_elem = doc.FirstChildElement("robot");
  if (!robot_elem) {
    RCLCPP_ERROR(logger, "No <robot> element found in robot_description");
    return false;
  }

  const tinyxml2::XMLElement* ros2_control_elem = robot_elem->FirstChildElement("ros2_control");
  if (!ros2_control_elem ||
      std::string(ros2_control_elem->Attribute("name")) != "FrankaHardwareInterface") {
    RCLCPP_ERROR(logger, "No FrankaHardwareInterface <ros2_control> section found");
    return false;
  }

  joint_interfaces.clear();
  for (const tinyxml2::XMLElement* joint_elem = ros2_control_elem->FirstChildElement("joint");
       joint_elem != nullptr; joint_elem = joint_elem->NextSiblingElement("joint")) {
    const char* joint_name = joint_elem->Attribute("name");
    if (!joint_name) {
      RCLCPP_WARN(logger, "Skipping joint with no name attribute");
      continue;
    } else {
      RCLCPP_DEBUG(logger, "Found joint: %s", joint_name);
    }

    JointControlInterfaces interfaces;
    for (const tinyxml2::XMLElement* cmd_elem = joint_elem->FirstChildElement("command_interface");
         cmd_elem != nullptr; cmd_elem = cmd_elem->NextSiblingElement("command_interface")) {
      const char* cmd_name = cmd_elem->Attribute("name");
      if (cmd_name) {
        interfaces.command_interfaces.emplace_back(cmd_name);
        RCLCPP_DEBUG(logger, "Adding joint %s command_interface: %s", joint_name, cmd_name);
      }
    }
    for (const tinyxml2::XMLElement* state_elem = joint_elem->FirstChildElement("state_interface");
         state_elem != nullptr; state_elem = state_elem->NextSiblingElement("state_interface")) {
      const char* state_name = state_elem->Attribute("name");
      if (state_name) {
        interfaces.state_interfaces.emplace_back(state_name);
        RCLCPP_DEBUG(logger, "Adding joint %s state_interface: %s", joint_name, state_name);
      }
    }

    if (!interfaces.command_interfaces.empty() || !interfaces.state_interfaces.empty()) {
      joint_interfaces[joint_name] = interfaces;
      RCLCPP_DEBUG(logger, "Extracted joint '%s': %zu command, %zu state interfaces", joint_name,
                   interfaces.command_interfaces.size(), interfaces.state_interfaces.size());
    }
  }

  if (joint_interfaces.empty()) {
    RCLCPP_ERROR(logger, "No valid joints found in <ros2_control> section");
    return false;
  }
  return true;
}

}  // namespace robot_utils