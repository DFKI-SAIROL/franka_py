std_msgs/Header header

geometry_msgs/Pose actual_pose
geometry_msgs/Pose target_pose
geometry_msgs/Pose safe_target_pose
geometry_msgs/Pose cmd_pose
geometry_msgs/Pose cmd_final_pose
geometry_msgs/Pose tf_pose

float32 safety_distance
float32 safety_distance_along_velocity

geometry_msgs/Twist cartesian_velocity

float64[] q_actual
float64[] dq_commanded
float64[] q_target
