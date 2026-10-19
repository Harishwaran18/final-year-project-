# One actor reported by the CARLA PythonAPI, expressed in the CARLA
# left-handed world frame. The bridge converts it into the ROS REP-103 frame.

std_msgs/Header header

uint32 actor_id

# CARLA blueprint-derived class, e.g. "vehicle", "walker", "traffic_light".
string class_label

# Actor pose in the CARLA world frame (x forward, y right, z up, metres).
geometry_msgs/Point position
geometry_msgs/Vector3 rotation

# Half-extents of the actor's bounding box (CARLA `extent`), metres.
geometry_msgs/Vector3 extent

# Actor velocity in the CARLA world frame [m/s].
geometry_msgs/Vector3 velocity
