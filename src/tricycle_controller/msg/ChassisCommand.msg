# Software kinematic command, never a measurement or permission to actuate.
# header.stamp: output computation time; frame_id: base_link direction convention.
std_msgs/Header header
# Stamped input's acquisition time; zero for plain Twist (receipt watchdog used).
time source_stamp
# Signed rear-axle midpoint body longitudinal speed, m/s. Forward positive.
float64 speed
# Front wheel steering, radians. Straight=0, left positive. Limit +/-50 deg.
float64 steering_angle
# Achievable body yaw rate after speed/steering saturation, rad/s.
float64 yaw_rate
# false for invalid, unsupported, waiting or timed-out input. Outputs then zero.
bool accepted
bool saturated
string reason
