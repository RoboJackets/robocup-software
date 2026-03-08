# A Control Command created by the controlle detailing how a robot should move

uint8 SHOOT_MODE_KICK = 0
uint8 SHOOT_MODE_CHIP = 1

uint8 TRIGGER_MODE_STAND_DOWN = 0
uint8 TRIGGER_MODE_IMMEDIATE = 1
uint8 TRIGGER_MODE_ON_BREAK_BEAM = 2

# How should the robot kick the ball
uint8 shoot_mode
# How should the kicker be actuated
uint8 trigger_mode
# How much power should be used to kick [0, 1]
float32 kick_strength
# How fast should the dribbler be moving ([-1, 1]) with -1 being away from us and 1 being toward us
float32 dribble_speed
# The (x, y, w) velocity the robot should move at
rj_geometry_msgs/Twist velocity