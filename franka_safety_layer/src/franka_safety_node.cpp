#include "franka_safety_layer/franka_safety_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace franka_safety_layer
{

SafetyNode::SafetyNode() : Node("safety_node")
{
  // ROS I/O
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "target_cartesian_pose", 10, std::bind(&SafetyNode::targetPoseCallback, this, _1));

  joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("franka_joint_trajectory_controller/joint_trajectory", 10);

  RCLCPP_INFO(this->get_logger(), "SafetyNode constructed. Listening on '%s'.", target_pose_sub_->get_topic_name());
}

void SafetyNode::init()
{
  RCLCPP_INFO(this->get_logger(), "1");

  move_group_name_ = this->declare_parameter<std::string>("move_group", "fr3");
  
  // MoveIt setup
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), move_group_name_);
  move_group_->setPlanningTime(3.0);
  move_group_->setMaxVelocityScalingFactor(0.4);
  move_group_->setMaxAccelerationScalingFactor(0.4);

  RCLCPP_INFO(this->get_logger(), "2");

  // Planning scene monitor for collision checking
  planning_scene_monitor_ =
    std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
  if (planning_scene_monitor_->getPlanningScene())
  {
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize PlanningSceneMonitor!");
  }

  RCLCPP_INFO(this->get_logger(), "Initilaization completed");

}

void SafetyNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  trajectory_msgs::msg::JointTrajectory trajectory;

  if (planSafeTrajectory(*msg, trajectory))
  {
    RCLCPP_INFO(this->get_logger(), "Publishing safe joint trajectory.");
    joint_traj_pub_->publish(trajectory);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Failed to plan a safe trajectory.");
  }
}

bool SafetyNode::planSafeTrajectory(const geometry_msgs::msg::PoseStamped &target_pose, trajectory_msgs::msg::JointTrajectory &trajectory)
{
  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::core::MoveItErrorCode success = move_group_->plan(plan);

  if (success != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(this->get_logger(), "MoveIt planning failed.");
    return false;
  }

  // Safety check 1: velocity limits
  for (const auto &pt : plan.trajectory.joint_trajectory.points)
  {
    for (double vel : pt.velocities)
    {
      if (std::abs(vel) > 1.0)
      {
        RCLCPP_WARN(this->get_logger(), "Unsafe velocity detected, rejecting plan.");
        return false;
      }
    }
  }

  // Safety check 2: collision validation
  if (!isTrajectoryCollisionFree(plan.trajectory.joint_trajectory))
  {
    RCLCPP_WARN(this->get_logger(), "Planned trajectory is in collision!");
    return false;
  }

  trajectory = plan.trajectory.joint_trajectory;
  return true;
}


bool SafetyNode::isTrajectoryCollisionFree(const trajectory_msgs::msg::JointTrajectory &trajectory)
{
  if (!planning_scene_monitor_ || !planning_scene_monitor_->getPlanningScene())
  {
    RCLCPP_WARN(this->get_logger(), "Planning scene not available, skipping collision check.");
    return true;
  }

  // ✅ Lock the planning scene for read-only access
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);

  // ✅ Use correct MoveIt namespace: moveit::core
  moveit::core::RobotState state(scene->getCurrentState());
  const moveit::core::JointModelGroup *joint_model_group =
    state.getJointModelGroup(move_group_name_);

  if (!joint_model_group)
  {
    RCLCPP_ERROR(this->get_logger(), "JointModelGroup '%s' not found!", move_group_name_.c_str());
    return false;
  }

  for (const auto &point : trajectory.points)
  {
    state.setJointGroupPositions(joint_model_group, point.positions);

    // ✅ Update transforms before checking collisions
    state.update();

    if (scene->isStateColliding(state, move_group_name_, true))
    {
      RCLCPP_WARN(this->get_logger(), "Collision detected in trajectory!");
      return false;
    }
  }

  return true;
}


}  // namespace franka_safety_layer

// ---- main ----
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<franka_safety_layer::SafetyNode>();
  node->init();
  RCLCPP_INFO(node->get_logger(), "1+");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
