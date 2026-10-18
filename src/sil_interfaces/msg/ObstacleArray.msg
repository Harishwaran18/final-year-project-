# Set of fused obstacles for one fusion cycle.
std_msgs/Header header

# Monotonic fusion cycle counter, used for latency instrumentation.
uint64 cycle_id

# Wall-clock duration of the fusion step [ms]; budget is 18 ms.
float64 latency_ms

Obstacle[] obstacles
