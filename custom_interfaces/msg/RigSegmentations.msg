std_msgs/Header header

# Metadata: Maps Pixel IDs to Strings
# e.g. segmentation_labels[0] = "background", segmentation_labels[1] = "mug"
string[] segmentation_labels

# One mask per camera (Batch size = 3)
# Encoding: "mono8" (< 255 objects)
sensor_msgs/Image[] segmentation_masks