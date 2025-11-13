# Represents a single frontier for task allocation

# Unique identifier for this frontier
int32 id

# Robot that discovered this frontier
string robot_id

# Frontier geometric properties
uint32 size                          # Number of cells in frontier
geometry_msgs/Point centroid         # Center point of frontier
geometry_msgs/Point initial          # Initial contact point
geometry_msgs/Point middle           # Closest point to robot

# Frontier metrics
float64 min_distance                 # Distance from robot to frontier (meters)
float64 information_gain             # Expected information gain (unknown cells visible)
float64 cost                         # Combined cost metric (lower = better)

# All points that make up the frontier
geometry_msgs/Point[] points

# Timestamp
std_msgs/Header header
