# All actors visible to the CARLA simulator for one tick.
std_msgs/Header header

# Ego vehicle pose in the CARLA world frame, used as the transform origin.
geometry_msgs/Point ego_position
geometry_msgs/Vector3 ego_rotation

# Ego vehicle velocity in the CARLA world frame [m/s].
geometry_msgs/Vector3 ego_velocity

CarlaActor[] actors
