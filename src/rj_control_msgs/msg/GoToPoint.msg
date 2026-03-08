# Go to a specified point

# The point to move to
rj_geometry_msgs/Point target
# Should the controller treat the ball as an obstacle
bool avoid_ball
# The tolerance (in meters) from the target point where we consider the action complete
float64 tolerance