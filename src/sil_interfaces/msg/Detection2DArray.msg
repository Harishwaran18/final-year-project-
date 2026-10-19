# All 2D detections for one camera frame.
std_msgs/Header header

# Camera intrinsics so the fusion node can convert pixels to bearings.
float64 fx
float64 fy
float64 cx
float64 cy

uint32 image_width
uint32 image_height

Detection2D[] detections
