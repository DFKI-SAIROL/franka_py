#include <chrono>
#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::chrono_literals;

class FrankaCartesianModule : public rclcpp::Node, public std::enable_shared_from_this<FrankaCartesianModule>
{
public:
    FrankaCartesianModule()
    : Node("franka_cartesian_module")
    {

        this->declare_parameter("arm_prefix", "franka_undefined");
        arm_prefix_ = this->get_parameter("arm_prefix").as_string();
        if(arm_prefix_ != "") 
        {
            arm_prefix_ += "_";
        }

        end_effector_link_ = arm_prefix_ + "fr3_link8";

        this->declare_parameter("init_joint_position", std::vector<double>(7, 0.0));
        init_joint_position_ = this->get_parameter("init_joint_position").as_double_array();

        if (std::all_of(init_joint_position_.begin(), init_joint_position_.end(),
                        [](double x){ return x == 0.0; }))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid init joint positions. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Publisher
        cartesian_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_cartesian_pose", 10);

        // Timer for publishing Cartesian motion
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 15), std::bind(&FrankaCartesianModule::publish_cartesian_motion, this));

        RCLCPP_INFO(this->get_logger(), "Franka Cartesian module started");
    }

    void init(std::shared_ptr<FrankaCartesianModule> node)
    {
        // Create parameter client to get robot_description
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "robot_state_publisher");

        RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher parameter server...");
        while (!param_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher to be available...");
        }

        // Wait until the parameter exists
        while (!param_client->has_parameter("robot_description")) {
            RCLCPP_INFO(this->get_logger(), "Waiting for 'robot_description' parameter...");
            rclcpp::sleep_for(500ms);
        }

        std::string urdf_string = param_client->get_parameter<std::string>("robot_description");
        if (urdf_string.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty robot_description from robot_state_publisher.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully fetched robot_description parameter from robot_state_publisher");

        // Load Pinocchio model from URDF string
        try {
            pinocchio::urdf::buildModelFromXML(urdf_string, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF from parameter: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        if (!model_.existFrame(end_effector_link_)) {
            RCLCPP_ERROR(this->get_logger(), "End effector link '%s' not found in model.", end_effector_link_.c_str());
            rclcpp::shutdown();
            return;
        }
        ee_frame_id_ = model_.getFrameId(end_effector_link_);

        Eigen::VectorXd q = Eigen::Map<Eigen::VectorXd>(init_joint_position_.data(), init_joint_position_.size());

        // Compute FK
        pinocchio::forwardKinematics(model_, *data_, q);
        pinocchio::updateFramePlacements(model_, *data_);
        const pinocchio::SE3 &ee_pose = data_->oMf[ee_frame_id_];

        Eigen::Isometry3d ee_iso = Eigen::Isometry3d::Identity();
        ee_iso.linear() = ee_pose.rotation();
        ee_iso.translation() = ee_pose.translation();

        init_pose_.header.frame_id = "base";
        init_pose_.pose = tf2::toMsg(ee_iso);

        RCLCPP_INFO(this->get_logger(), "Initial EE pose computed from robot_description (Pinocchio): %.3f %.3f %.3f", init_pose_.pose.position.x, init_pose_.pose.position.y, init_pose_.pose.position.z);
        init_pose_calculated_ = true;
        start_time_ = this->get_clock()->now();
    }


private:
    void publish_cartesian_motion()
    {
        if(!init_pose_calculated_)
        {
            return;
        }

        RCLCPP_INFO_ONCE(this->get_logger(), "Start publish cartesian target poses.");
        double elapsed = (this->get_clock()->now() - start_time_).seconds();

        // --- Parameters (Add new ones for rotation) ---
        double offset_x = 0.0;
        double offset_z = -0.3;
        double amplitude_x = 0.1;
        double amplitude_z = 0.13;
        double frequency_pos = 0.05; // Renamed for clarity: Position frequency

        double offset_y = -0.1;
        double amplitude_y = 0.2;
        double frequency_y = 0.03; // Renamed for clarity: Position frequency

        // New parameters for rotation
        double amplitude_yaw = M_PI / 4.0; // 45 degrees maximum rotation around Z-axis
        double amplitude_roll = M_PI / 10.0; // 18 degrees maximum rotation around Z-axis
        double frequency_angle = 0.02;       // Slower frequency for rotation

        geometry_msgs::msg::PoseStamped pose = init_pose_;
        pose.header.stamp = this->get_clock()->now();

        // --- 1. Compute Position (X and Z) ---
        pose.pose.position.x = init_pose_.pose.position.x + offset_x + amplitude_x * std::sin(2.0 * M_PI * frequency_pos * elapsed);
        pose.pose.position.y = init_pose_.pose.position.y + offset_y + amplitude_y * std::cos(2.0 * M_PI * frequency_y * elapsed);
        pose.pose.position.z = init_pose_.pose.position.z + offset_z + amplitude_z * std::cos(2.0 * M_PI * frequency_pos * elapsed);

        // --- 2. Compute Euler Angle (Yaw) ---
        double yaw_angle = amplitude_yaw * std::sin(2.0 * M_PI * frequency_angle * elapsed);
        double roll_angle = amplitude_roll * std::sin(2.0 * M_PI * frequency_angle * elapsed);
        double pitch_angle = M_PI;

        // --- 3. Convert Euler Angles to Quaternion ---
        tf2::Quaternion q;
        // The order is Roll, Pitch, Yaw
        q.setRPY(roll_angle, pitch_angle, yaw_angle);

        // --- 4. Assign Quaternion to Pose Message ---
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        cartesian_pose_pub_->publish(pose);
    }

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    // Pinocchio model
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex ee_frame_id_;

    // Parameters
    std::string end_effector_link_, arm_prefix_;
    std::vector<double> init_joint_position_;
    geometry_msgs::msg::PoseStamped init_pose_;
    bool init_pose_calculated_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrankaCartesianModule>();
    node->init(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
