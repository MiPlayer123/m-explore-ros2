# Array of frontiers from a single robot

# Header with timestamp and frame_id
std_msgs/Header header

# Robot identifier (namespace)
string robot_id

# Array of frontiers discovered by this robot
Frontier[] frontiers
