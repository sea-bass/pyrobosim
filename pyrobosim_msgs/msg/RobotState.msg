# Robot state definition message

# Robot information
string name

# Continuous state
geometry_msgs/Pose pose

# Discrete state
bool executing_action
bool holding_object
string manipulated_object
string last_visited_location
