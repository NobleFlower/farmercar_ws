std_msgs/Header header
bool received
bool fresh
bool simulated
uint8 protocol_version
# SI speed and front steering, calibrated from the raw protocol units.
float64 speed
float64 steering_angle
# False for legacy setting feedback unless explicitly configured as measured.
bool speed_is_measured
bool steering_is_measured
# Only extended status can authoritatively report these hardware fields.
bool status_authoritative
bool controller_ready
bool emergency_stop
# 0 disabled, 1 manual, 2 automatic (extended).
uint8 control_mode
uint16 fault_flags
bool ack_valid
uint16 ack_sequence
