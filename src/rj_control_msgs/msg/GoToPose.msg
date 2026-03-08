# Go to a specified pose

# The pose to get to
rj_geometry_msgs/Pose target
# Should the controller treat the ball as an obstacle
bool avoid_ball
# The position tolerance (in meters)
float64 position_tolerance
# The angular tolerance (in radians)
float64 angular_tolerance