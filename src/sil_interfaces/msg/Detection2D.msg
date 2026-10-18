# A single 2D object detection produced by the CARLA camera hook or by an
# on-vehicle camera detector. Consumed by the sensor fusion node.

std_msgs/Header header

uint32 detection_id

# Normalised class label, e.g. "car", "truck", "pedestrian".
string class_label

# Axis-aligned bounding box in pixel coordinates of the source image.
uint32 x_offset
uint32 y_offset
uint32 width
uint32 height

# Detector confidence in [0, 1].
float32 score
