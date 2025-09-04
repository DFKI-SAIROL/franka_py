#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class FrankaControlModule : public rclcpp::Node
{
public:
    FrankaControlModule()
    : Node("franka_control_module")
    {
        this->declare_parameter("arm_id", "fr3");
        this->declare_parameter("arm_prefix", "franka_undefined");

        arm_id_ = this->get_parameter("arm_id").as_string();
        arm_prefix_ = this->get_parameter("arm_prefix").as_string();
        if(arm_prefix_ != "") 
        {
            arm_prefix_ += "_";
        }

        start_time = this->get_clock()->now();

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "franka_joint_trajectory_controller/joint_trajectory", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&FrankaControlModule::publish_trajectory, this));

        RCLCPP_INFO(this->get_logger(), "Franka control module started.");

    }

private:
    void publish_trajectory()
    {

        auto msg = trajectory_msgs::msg::JointTrajectory();

        for(int i = 1; i <= 7; i++)
        {
            msg.joint_names.push_back(/*arm_prefix_ +*/ arm_id_ + "_joint" + std::to_string(i));
        }
        
        double duration = (this->get_clock()->now() - start_time).seconds();
        double value = 0.5 + 0.2 * std::sin(duration);

        // Define a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, value};
        point.time_from_start = rclcpp::Duration(1s);

        msg.points.push_back(point);

        RCLCPP_INFO_ONCE(this->get_logger(), "Publishing trajectory command.");
        publisher_->publish(msg);
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time;

    std::string arm_id_;
    std::string arm_prefix_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankaControlModule>());
    rclcpp::shutdown();
    return 0;
}
