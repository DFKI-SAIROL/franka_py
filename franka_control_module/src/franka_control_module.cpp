#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class FrankaControlModule : public rclcpp::Node
{
public:
    FrankaControlModule()
    : Node("franka_control_module")
    {
        this->declare_parameter("arm_id", "fr3");
        this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));

        arm_id_ = this->get_parameter("arm_id").as_string();
        
        init_joint_position_ = this->get_parameter("init_joint_position").as_double_array();
        if(std::all_of(init_joint_position_.begin(), init_joint_position_.end(), [](double x){ return x == 0.0; }))
        { 
            RCLCPP_INFO(this->get_logger(), "Invalid init joint position, shutdown");
            exit(-1);
        }

        if(init_joint_position_[6] == 0.8) // left
        {
            periodic_length_factor = 0.4;
        }

        start_time_ = this->get_clock()->now();

        joint_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "franka_joint_trajectory_controller/joint_trajectory", 1);

        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&FrankaControlModule::jointStateCallback, this, std::placeholders::_1));
        

        timer_ = this->create_wall_timer(20ms, std::bind(&FrankaControlModule::publish_trajectory, this));

        RCLCPP_INFO(this->get_logger(), "Franka control module started.");

    }

private:

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

    }

    void publish_trajectory()
    {

        auto msg = trajectory_msgs::msg::JointTrajectory();
        msg.header.stamp = this->get_clock()->now();
        for(int i = 1; i <= 7; i++)
        {
            msg.joint_names.push_back(arm_id_ + "_joint" + std::to_string(i));
        }

        if(!initialized_)
        {

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = init_joint_position_;
            point.velocities = std::vector<double>(7, 0.0);

            point.time_from_start = rclcpp::Duration(2000ms);
            msg.points.push_back(point);

            RCLCPP_INFO_ONCE(this->get_logger(), "Publishing init trajectory command.");
            joint_trajectory_publisher_->publish(msg);

            if((this->get_clock()->now() - start_time_).seconds() >= 1)
            {
                initialized_ = true;
            }

        }
        else
        {
            if(initialized_ && (this->get_clock()->now() - start_time_).seconds() >= init_time_s)
            {
                double duration = (this->get_clock()->now() - start_time_ - rclcpp::Duration(init_time_s, 0)).seconds();
                double value = 1 - movement_factor * std::cos(periodic_length_factor * duration);
                double dvalue = movement_factor * periodic_length_factor * std::sin(periodic_length_factor * duration);

                // Define a trajectory point
                {
                    trajectory_msgs::msg::JointTrajectoryPoint point;

                    point.positions = init_joint_position_;
                    point.velocities = std::vector<double>(7, 0.0);

                    point.positions[6] += value;
                    point.velocities[6] += dvalue;

                    point.time_from_start = rclcpp::Duration(trajectory_points_time);

                    msg.points.push_back(point);
                }

                // Define a trajectory point
                /*{
                    trajectory_msgs::msg::JointTrajectoryPoint point;

                    point.positions = init_joint_position_;
                    point.velocities = std::vector<double>(7, 0.0);

                    point.positions[6] += value + dvalue * rclcpp::Duration(trajectory_points_time).seconds();
                    point.velocities[6] += dvalue;

                    point.time_from_start = rclcpp::Duration(2*trajectory_points_time);

                    msg.points.push_back(point);
                }*/

                RCLCPP_INFO_ONCE(this->get_logger(), "Publishing periodic trajectory command.");
                joint_trajectory_publisher_->publish(msg);
            }
        }

    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;
    bool initialized_ = false;

    std::string arm_id_;
    std::vector<double> init_joint_position_;

    double periodic_length_factor = 0.5;
    double movement_factor = 1;
    int init_time_s = 5;
    std::chrono::milliseconds trajectory_points_time = 100ms;
    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrankaControlModule>());
    rclcpp::shutdown();
    return 0;
}
