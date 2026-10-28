# Candidate velocity for an adapter, never a physical stop acknowledgement.
std_msgs/Header header
string task_id
string execution_id
string segment_id
string phase
string mode
# TRACK: valid velocity; HOLD: revoke velocity and request adapter hold/exit policy.
string behavior
float64 vx
float64 wz
bool valid
builtin_interfaces/Time valid_until
