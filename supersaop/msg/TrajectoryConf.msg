# Trajectory, UAV, parameters for the mission planner

# trajectory name, unique within a plan
string name

# name of the uav model, e.g. "x8-06"
string uav_model

# base start and end
PoseEuler start_wp
PoseEuler end_wp

time start_time
float64 max_duration

MeanWind wind
