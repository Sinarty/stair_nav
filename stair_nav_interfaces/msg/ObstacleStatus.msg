# Fresh perception decision. An invalid/expired observation never grants motion.
std_msgs/Header header
builtin_interfaces/Time valid_until
string execution_id
# Stamp of the reference path inspected; zero before any path is available.
builtin_interfaces/Time path_stamp
bool valid
bool blocked
string reason
float64 nearest_distance
uint32 occupied_voxels
