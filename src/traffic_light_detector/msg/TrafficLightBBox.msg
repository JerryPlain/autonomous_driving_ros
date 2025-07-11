# Single traffic light bounding box with distance
std_msgs/Header header
int32       id        # Connected component label (unique ID)
int32       x         # Top-left x coordinate in pixels
int32       y         # Top-left y coordinate in pixels
int32       width     # Width of the bounding box in pixels
int32       height    # Height of the bounding box in pixels
float32     distance  # Median distance to the traffic light in meters