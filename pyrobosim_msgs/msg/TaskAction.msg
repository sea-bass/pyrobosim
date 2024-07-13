# Task Action ROS Message

# Main action information
string robot
string type
string object
string room
string source_location
string target_location

# Action cost (from the output of a planner)
float32 cost

# Action execution options (e.g., delay and success probability)
ActionExecutionOptions execution_options

# Other parameters
bool has_pose
geometry_msgs/Pose pose
pyrobosim_msgs/Path path
