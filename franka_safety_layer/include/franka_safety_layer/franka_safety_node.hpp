#ifndef FRANKA_SAFETY_LAYER__SAFETY_NODE_HPP_
#define FRANKA_SAFETY_LAYER__SAFETY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace franka_safety_layer
{

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode();
  void init();

private:
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool planSafeTrajectory(
    const geometry_msgs::msg::PoseStamped &target_pose,
    trajectory_msgs::msg::JointTrajectory &trajectory);
  bool isTrajectoryCollisionFree(const trajectory_msgs::msg::JointTrajectory &trajectory);

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_traj_pub_;

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  // Parameters
  std::string move_group_name_;
  std::string trajectory_topic_;
};

}  // namespace franka_safety_layer

#endif  // FRANKA_SAFETY_LAYER__SAFETY_NODE_HPP_
