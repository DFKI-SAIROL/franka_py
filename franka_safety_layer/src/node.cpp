#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("trajectory_publisher_node");

class TrajectoryPublisherNode : public rclcpp::Node
{
public:
    TrajectoryPublisherNode() : Node("trajectory_publisher_node")
    {
        // 1. Initialisierung der MoveGroupInterface
        // "fr3_arm" ist der typische Name der Planungsgruppe für den Franka Arm.
        arm_move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "fr3_arm");

        // 2. Initialisierung des Publishers
        // Das Topic MUSS dem Command Topic Ihres joint_trajectory_controllers entsprechen.
        // Der Standardname in franka_ros2 ist oft /<controller_name>/joint_trajectory
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/fr3_arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());

        // 3. Start des Planungs- und Ausführungsprozesses nach kurzer Verzögerung
        RCLCPP_INFO(LOGGER, "MoveGroupInterface initialisiert. Starte Planung in 3 Sekunden...");
        
        timer_ = this->create_wall_timer(
            3s, std::bind(&TrajectoryPublisherNode::execute_motion, this));
    }

private:
    void execute_motion()
    {
        // Timer stoppen, da die Ausführung einmalig ist
        timer_->cancel();
        
        // --- 1. Kartesisches Ziel setzen ---
        geometry_msgs::msg::Pose target_pose;
        // ACHTUNG: Diese Pose MUSS in den Arbeitsraum des FR3 passen!
        target_pose.orientation.w = 0.707;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.707;
        target_pose.orientation.z = 0.0;
        target_pose.position.x = 0.55; // 55 cm in X
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.4;  // 40 cm in Z
        
        arm_move_group_interface_->setPoseTarget(target_pose);

        // --- 2. Planung durchführen (IK & Kollisionsprüfung) ---
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(LOGGER, "Planung erfolgreich. Kollisionsfreier Pfad gefunden.");

            // --- 3. Manuelle Trajektorien-Ausführung (Ihr Safety-Layer) ---
            RCLCPP_INFO(LOGGER, "Starte manuelle Ausführung der Trajektorie...");
            
            const auto& trajectory = plan.trajectory.joint_trajectory;
            size_t num_points = trajectory.points.size();

            for (size_t i = 0; i < num_points; ++i)
            {
                const auto& point = trajectory.points[i];
                
                // Erstellt eine JointTrajectory-Nachricht, die nur diesen einen Punkt enthält.
                // Der joint_trajectory_controller kann diese auch verarbeiten.
                trajectory_msgs::msg::JointTrajectory point_msg;
                point_msg.header.stamp = this->now(); // Aktueller Zeitstempel
                point_msg.joint_names = trajectory.joint_names;
                point_msg.points.push_back(point);
                
                // **HIER IST IHR SICHERHEITSLAYER:**
                // Fügen Sie hier Ihre benutzerdefinierte Logik ein, z.B.:
                // - if (sensor_daten_kollision): return;
                // - Geschwindigkeitslimitierung: point_msg.points[0].velocities anpassen

                // Publizieren des Punktes
                trajectory_pub_->publish(point_msg);
                
                RCLCPP_DEBUG(LOGGER, "Published point %zu/%zu", i + 1, num_points);
                
                // Wartezeit bis zum nächsten Punkt
                if (i < num_points - 1) {
                    // Berechnung der Zeitdifferenz zwischen dem aktuellen und dem nächsten Punkt
                    rclcpp::Duration duration = rclcpp::Duration(
                        plan.trajectory.joint_trajectory.points[i+1].time_from_start.sec,
                        plan.trajectory.joint_trajectory.points[i+1].time_from_start.nanosec) - rclcpp::Duration(
                        point.time_from_start.sec,
                        point.time_from_start.nanosec);
                        
                    // Umwandlung in Nanosekunden und Pausieren des Threads
                    std::this_thread::sleep_for(std::chrono::nanoseconds(duration.nanoseconds()));
                } else {
                    // Kurze Wartezeit nach dem letzten Punkt, bevor der Knoten beendet wird
                    std::this_thread::sleep_for(500ms);
                }
            }
            RCLCPP_INFO(LOGGER, "Trajektorien-Ausführung abgeschlossen.");

        } else {
            RCLCPP_ERROR(LOGGER, "Planung fehlgeschlagen. Ziel nicht erreichbar oder kollidiert.");
        }
        
        // Nach Abschluss die ROS-Schleife beenden.
        rclcpp::shutdown();
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_interface_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryPublisherNode>();
    rclcpp::spin(node);
    return 0;
}