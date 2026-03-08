# The action to take

uint8 GO_TO_POINT = 0
uint8 GO_TO_POSE = 1
uint8 SHOOT = 2
uint8 COLLECT = 3
uint8 PASS = 4
uint8 DRIBBLE = 5
uint8 CLEAR = 6
uint8 MARK_ROBOT = 7
uint8 ROTATE_TO_HEADING = 8
uint8 UNKNOWN = 9

# The action to take
uint8 action

# Go to a point ignoring header
GoToPoint[<=1] go_to_point
# Go to a pose using the header
GoToPose[<=1] go_to_pose
# Shoot the ball at the goal
Shoot[<=1] shoot
# Collect the ball
Collect[<=1] collect
# Pass the ball
Pass[<=1] pass_ball
# Dribble the ball
Dribble[<=1] dribble
# Clear the ball
Clear[<=1] clear
# Mark the other team's robot
MarkRobot[<=1] mark_robot
# Rotate to a given heading
RotateToHeading[<=1] rotate_to_heading