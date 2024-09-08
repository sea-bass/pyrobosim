# Robot state definition message

# Header containing a timestamp
std_msgs/Header header

# Robot information
string name

# Continuous state
geometry_msgs/Pose pose
float64 battery_level

# Discrete state
bool executing_action
bool holding_object
string manipulated_object
string last_visited_location
