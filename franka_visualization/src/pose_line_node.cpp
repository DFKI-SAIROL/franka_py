#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <franka_custom_msgs/msg/fijk_debug.hpp>

// --- TF2 Includes ---
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // Required for transforming geometry_msgs::msg::PoseStamped

#include <unordered_map>
#include <deque>
#include <cmath>
#include <string>
#include <vector>
#include <random>

using std::placeholders::_1;

struct PoseLine {
  std::deque<geometry_msgs::msg::Point> points;
  geometry_msgs::msg::Point last;
  bool has_last = false;
  std::array<float,3> color;
};

class PoseLineNode : public rclcpp::Node {
public:
  PoseLineNode() : Node("pose_line_node"),
                   // Initialize TF2 components
                   tf_buffer_(this->get_clock()),
                   tf_listener_(tf_buffer_)
  {
    declare_parameter<int>("line_length", 500);
    declare_parameter<double>("min_distance", 0.001);

    line_length_ = get_parameter("line_length").as_int();
    min_distance_ = get_parameter("min_distance").as_double();

    publisher_arrow_ = create_publisher<visualization_msgs::msg::Marker>("vis_arrow", 10);
    publisher_line_ = create_publisher<visualization_msgs::msg::MarkerArray>("vis_line", 10);

    // Setup for the first subscription (original)
    sub_1 = create_subscription<geometry_msgs::msg::PoseStamped>("target_cartesian_pose", 10, std::bind(&PoseLineNode::tcp_callback, this, _1));
    sub_2 = create_subscription<franka_custom_msgs::msg::FIJKDebug>("fijk_debug", 10, std::bind(&PoseLineNode::fijk_debug_callback, this, _1));

    // Detect own namespace
    std::string ns = this->get_namespace();

    // Default: use franka_right
    std::string arm = "franka_unknown";

    if (ns == "/franka_left") {
      arm = "franka_left";
    } 
    else if (ns == "/franka_right") {
      arm = "franka_right";
    } 
    else if (ns == "/franka_main") {
      arm = "franka_main";
    } 
    else {
      RCLCPP_WARN(this->get_logger(), "PoseLineNode started in namespace '%s' which is not a known arm.", ns.c_str());
      rclcpp::shutdown(); 
    }

    // Assign TF frames based on namespace
    source_frame_ = arm + "_fr3_link0";
    target_frame_ = arm + "_fr3_link8";

    // Setup timer for TF lookup (new)
    timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&PoseLineNode::tf_callback, this));

    RCLCPP_INFO(this->get_logger(), "PoseLineNode started.");
  }

private:

  void tcp_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = 1; 
    // ID 0, Namespace "r_tcp" (original)
    update(0, "r_tcp", r_tcp_line, msg->pose, color, msg->header);
  }

  void fijk_debug_callback(const franka_custom_msgs::msg::FIJKDebug::SharedPtr msg)
  {
    {
      std_msgs::msg::ColorRGBA color;
      color.b = 1; 
      update(2, "b_stcp", b_stcp_line, msg->safe_target_pose, color, msg->header);
    }
    {
      std_msgs::msg::ColorRGBA color;
      color.b = 1; 
      color.g = 1; 
      update(3, "o_stcp", o_cmd_line, msg->cmd_pose, color, msg->header);
    }
  }

  void tf_callback()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Attempt to lookup the transform
    try {
      // Get the transform from source_frame_ to target_frame_ at the latest time
      transformStamped = tf_buffer_.lookupTransform(
        source_frame_, target_frame_,
        tf2::TimePointZero // Use TimePointZero for the latest available transform
      );
    } catch (const tf2::TransformException & ex) {
      // Log error but continue
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000, // Throttle to 1 second
        "Could not transform %s to %s: %s",
        target_frame_.c_str(), source_frame_.c_str(), ex.what());
      return;
    }

    // Convert the TransformStamped into a Pose
    geometry_msgs::msg::Pose pose;
    pose.position.x = transformStamped.transform.translation.x;
    pose.position.y = transformStamped.transform.translation.y;
    pose.position.z = transformStamped.transform.translation.z;
    pose.orientation = transformStamped.transform.rotation;
    
    // Define color for the TF visualization (e.g., green)
    std_msgs::msg::ColorRGBA color;
    color.g = 1; 
    
    // ID 1, Namespace "tf_pose" (new)
    update(1, "g_acp", tf_pose_line, pose, color, transformStamped.header);
  }

  void update(int id, std::string ns, PoseLine &line, geometry_msgs::msg::Pose &pose, std_msgs::msg::ColorRGBA &color, std_msgs::msg::Header &header) 
  {
    // --- Arrow Visualization ---
    visualization_msgs::msg::Marker arrow;
    arrow.header = header;
    arrow.ns = ns;
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.05;  
    arrow.scale.y = 0.02;  
    arrow.scale.z = 0.05;  
    arrow.color = color;
    arrow.color.a = 1;
    arrow.pose = pose;
   
    publisher_arrow_->publish(arrow);

    // --- Line Strip Visualization ---
    auto pt = pose.position;
    if (!line.has_last) {
      line.points.push_back(pt);
      line.last = pt;
      line.has_last = true;
      // Don't publish line strip until we have more than one point
      return;
    }

    double dx = pt.x - line.last.x;
    double dy = pt.y - line.last.y;
    double dz = pt.z - line.last.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist >= min_distance_) {
      line.points.push_back(pt);
      line.last = pt;
      if (line.points.size() > static_cast<size_t>(line_length_)) {
        line.points.pop_front();
      }
    }
   
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.005;  // thickness
    marker.color = color;
    marker.color.a = 0.5;

    marker.points.assign(line.points.begin(), line.points.end());
    array.markers.push_back(marker);

    publisher_line_->publish(array);
  }

  // Parameters
  int line_length_;
  double min_distance_;
  std::string target_frame_;
  std::string source_frame_;

  // State
  PoseLine r_tcp_line, tf_pose_line, o_cmd_line, b_stcp_line;

  // Ros
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_1;
  rclcpp::Subscription<franka_custom_msgs::msg::FIJKDebug>::SharedPtr sub_2;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_line_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2 Members (new)
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseLineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}