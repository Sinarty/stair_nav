# Ordered transition stream on /mission/events. Pair source_instance and
# sequence to identify events, detect gaps, and distinguish node restarts.
std_msgs/Header header
string source_instance
uint64 sequence
# MISSION_STATE or SEGMENT_STATE. Phase changes are carried in status.phase.
string kind
string old_state
string new_state
string event
string reason
string error_code
# Snapshot captured at this transition, not the state at publication time.
stair_nav_interfaces/MissionStatus status
