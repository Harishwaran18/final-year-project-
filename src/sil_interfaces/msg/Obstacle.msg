# A single fused dynamic obstacle tracked by the sensor fusion node.
# Frame convention: REP-103 (x forward, y left, z up), metres and m/s.

std_msgs/Header header

uint32 track_id

# Fused kinematic state (EKF posterior).
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# 2x2 position covariance, row-major (xx, xy, yx, yy).
float64[4] position_covariance

# Signed closing speed along the ego longitudinal axis [m/s].
# Positive == closing (obstacle approaching ego).
float64 closing_speed

# Euclidean range from the ego origin [m].
float64 range

# Mahalanobis gating distance used for the last data association.
float64 mahalanobis_distance

# Number of consecutive observations associated with this track.
uint32 hits

# True when the track was confirmed by both LiDAR and camera modalities.
bool multimodal
