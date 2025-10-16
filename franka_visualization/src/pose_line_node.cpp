#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>

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
  PoseLineNode() : Node("pose_line_node") 
  {
    declare_parameter<int>("line_length", 500);
    declare_parameter<double>("min_distance", 0.001);

    line_length_ = get_parameter("line_length").as_int();
    min_distance_ = get_parameter("min_distance").as_double();

    publisher_arrow_ = create_publisher<visualization_msgs::msg::Marker>("/vis_arrow", 10);
    publisher_line_ = create_publisher<visualization_msgs::msg::MarkerArray>("/vis_line", 10);

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.2f, 1.0f);

    sub_1 = create_subscription<geometry_msgs::msg::PoseStamped>("/franka_right/target_cartesian_pose", 10, std::bind(&PoseLineNode::r_tcp_callback, this, _1));

  }

private:

  void r_tcp_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {

    RCLCPP_INFO(this->get_logger(), "tcp cb %.2f %.2f %.2f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    std_msgs::msg::ColorRGBA color;
    color.r = 1;
    
    update(0, "r_tcp", r_tcp_line, msg->pose, color, msg->header);
  }

  void update(int id, std::string ns, PoseLine &line, geometry_msgs::msg::Pose &pose, std_msgs::msg::ColorRGBA &color, std_msgs::msg::Header &header) 
  {

    // array

    visualization_msgs::msg::Marker arrow;
    arrow.header = header;
    arrow.ns = ns;
    arrow.id = id;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.1;  
    arrow.scale.y = 0.05;  
    arrow.scale.z = 0.1;  
    arrow.color = color;
    arrow.color.a = 1;
    arrow.pose = pose;
   
    publisher_arrow_->publish(arrow);

    // line 
    auto pt = pose.position;
    if (!line.has_last) {
      line.points.push_back(pt);
      line.last = pt;
      line.has_last = true;
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
    marker.scale.x = 0.01;  // thickness
    marker.color = color;
    marker.color.a = 0.5;

    marker.points.assign(line.points.begin(), line.points.end());
    array.markers.push_back(marker);

    publisher_line_->publish(array);
  }

  // Parameters
  int line_length_;
  double min_distance_;

  // State
  PoseLine r_tcp_line;

  // Ros
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_1;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_arrow_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_line_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseLineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
