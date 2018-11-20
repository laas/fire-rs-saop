# Trajectory current completion state

Header header

# trajectory name
string name

# name of the uav, e.g. "x8-06"
string uav

# Waypoint the UAV is heading to
string maneuver

# Estimated Time of Arrival
duration maneuver_eta

# Allowed trajectory states
uint8 TRAJECTORY_STATE_BLOCKED=0
uint8 TRAJECTORY_STATE_READY=1
uint8 TRAJECTORY_STATE_EXECUTING=2

# trajectory current state
uint8 state

# Allowed trajectory states
uint8 TRAJECTORY_OUTCOME_NOTHING=0
uint8 TRAJECTORY_OUTCOME_SUCCESS=1
uint8 TRAJECTORY_OUTCOME_FAILURE=2

# last trajectory execution outcome state
uint8 last_outcome