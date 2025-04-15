# NavPath.msg

std_msgs/Header header

# Path as a list of points
project_interfaces/NavPoint[] path

# Attributes relating to navigating using path
float64 yaw
float64 velocity
bool reverse
