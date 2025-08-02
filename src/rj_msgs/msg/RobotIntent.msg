uint8 robot_id

uint8 SHOOT_MODE_KICK = 0
uint8 SHOOT_MODE_CHIP = 1

# Stand Down - Never Kick
# Immediate - Instantly Kick
# On Break Beam - Kick once break
# At End - Signals Planner to Kick after it is finished (planners do not support this by default)
uint8 TRIGGER_MODE_STAND_DOWN = 0
uint8 TRIGGER_MODE_IMMEDIATE = 1
uint8 TRIGGER_MODE_ON_BREAK_BEAM = 2
uint8 TRIGGER_MODE_AT_END = 3

# Off - Off throughout entire planner
# On - On throughout entire planner
# Default - Planner decides when to turn on dribbler
uint8 DRIBBLER_MODE_OFF = 0
uint8 DRIBBLER_MODE_ON = 1
uint8 DRIBBLER_MODE_DEFAULT = 2

MotionCommand motion_command

# Set of obstacles added by plays
rj_geometry_msgs/ShapeSet local_obstacles

uint8 shoot_mode
uint8 trigger_mode
uint8 dribbler_mode
float32 kick_speed

bool is_active

# Priority for motion planner
int8 priority
