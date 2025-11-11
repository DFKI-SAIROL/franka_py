#include "franka_safety_layer/franka_safety_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace franka_safety_layer
{

SafetyNode::SafetyNode() : Node("safety_node")
{
  target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "target_cartesian_pose", 10, std::bind(&SafetyNode::targetPoseCallback, this, _1));

  joint_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("franka_joint_trajectory_controller/joint_trajectory", 10);

  // TODO add services for plan & collision check

  plan_client_ = this->create_client<moveit_msgs::srv::GetMotionPlan>("plan_kinematic_path");

  while (!plan_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the plan_kinematic_path service. Exiting.");
      exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "plan_kinematic_path service not available, waiting again...");
  }

  RCLCPP_INFO(this->get_logger(), "SafetyNode constructed.");
}


void SafetyNode::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

  auto service_request =
      create_ik_service_request(new_position, orientation_, joint_positions_current_,
                                joint_velocities_current_, joint_efforts_current_);

  using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedFuture;
  auto response_received_callback =
      [&](ServiceResponseFuture future) {  // NOLINT(performance-unnecessary-value-param)
        const auto& response = future.get();

        if (response->error_code.val == response->error_code.SUCCESS) {
          joint_positions_desired_ = response->solution.joint_state.position;
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematics solution failed.");
        }
      };
  auto result_future_ =
      compute_ik_client_->async_send_request(service_request, response_received_callback);

  if (joint_positions_desired_.empty()) {
    return controller_interface::return_type::OK;
  }


}


std::shared_ptr<moveit_msgs::srv::GetMotionPlan::Request>
SafetyNode::create_plan_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    ) {
  auto service_request = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();

  service_request->
  return service_request;
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
  RCLCPP_INFO(node->get_logger(), "1+");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
