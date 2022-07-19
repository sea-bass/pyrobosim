# Task Action ROS Message

# Main Action information
string type
string object
string room
string source_location
string target_location

# Action cost (from the output of a planner)
float32 cost

# Other parameters
bool has_pose
geometry_msgs/Pose pose
pyrobosim_msgs/Path path
