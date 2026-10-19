# AEB safety alert published by the AEB controller FSM.

# FSM state enumeration. Keep in sync with sil_aeb_controller/aeb_fsm.hpp.
uint8 STATE_NORMAL = 0
uint8 STATE_FORWARD_COLLISION_WARNING = 1
uint8 STATE_PARTIAL_BRAKING = 2
uint8 STATE_FULL_EMERGENCY_BRAKE = 3

std_msgs/Header header

uint8 state

# Track that drove the current decision (0 == none).
uint32 target_track_id

# Time-to-collision for the selected target [s]. +inf when not closing.
float64 ttc

# Range to the selected target [m].
float64 distance

# Closing speed toward the selected target [m/s].
float64 closing_speed

# Deceleration requested from the brake system [m/s^2], >= 0.
float64 requested_deceleration

# Normalised brake pressure request in [0, 1].
float64 brake_pressure

# Set when the alert escalates the state on this cycle.
bool state_changed
