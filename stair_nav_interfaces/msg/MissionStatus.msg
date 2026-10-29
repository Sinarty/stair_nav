# Published heartbeat/snapshot time in the configured ROS clock and map frame.
std_msgs/Header header
# Snapshot lease. A cached terminal state is historical once this expires.
builtin_interfaces/Time valid_until
string task_id
string route_id
string execution_id
string segment_id
# 1-based configured segment ordinal; zero before a mission.
uint32 segment_index
uint32 segment_count
uint32 completed_segments
# Task-level state, independent of the downstream motion/execution phase.
string state
string phase
string segment_state
# Downstream route-progress value, not an overall distance percentage.
float64 segment_progress
string error_code
string error_message
bool navigation_stopped
bool hold_confirmed
bool blocked
bool cancel_requested
bool cancel_sent
bool cancel_accepted
bool downstream_terminal
# Hex ROS Action UUIDs; task goal is empty for the legacy Trigger interface.
string goal_id
string segment_goal_id
