std_msgs/Header header
string task_id
string execution_id
string segment_id
string phase
string state
float64 progress
float64 lateral_error
float64 heading_error
string error_code
string error_message
bool geometry_reached
# These are deliberately independent: invalid navigation commands != safe hold.
bool navigation_stopped
bool hold_confirmed
