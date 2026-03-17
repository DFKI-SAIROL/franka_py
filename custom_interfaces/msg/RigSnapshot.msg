std_msgs/Header header

# Metadata
string[] camera_names

# Intrinsics
sensor_msgs/CameraInfo[] cam_infos

# Visual Data
sensor_msgs/Image[] rgbs
sensor_msgs/Image[] depths