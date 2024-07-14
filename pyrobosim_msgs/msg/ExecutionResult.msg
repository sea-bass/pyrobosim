# Execution Result ROS Message

########################################
# Constant values for the status codes #
########################################
int32 UNKNOWN=-1

# Action executed successfully.
int32 SUCCESS=0

# Preconditions not sufficient to execute the action.
# For example, the action was to pick an object but there was no object visible.
int32 PRECONDITION_FAILURE=1

# Planning failed, for example a path planner or grasp planner did not produce a solution.
int32 PLANNING_FAILURE=2

# Preconditions were met and planning succeeded, but execution failed.
int32 EXECUTION_FAILURE=3

# Execution succeeded, but post-execution validation failed.
int32 POSTCONDITION_FAILURE=4

# Invalid action type.
int32 INVALID_ACTION=5

# The action was canceled by a user or upstream program.
int32 CANCELED=6

##################
# Message Fields #
##################
# The status code.
int32 status -1

# A message describing the result.
string message
