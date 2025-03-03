uint8 robot_id

uint8 SHOOT_MODE_KICK = 0
uint8 SHOOT_MODE_CHIP = 1

uint8 TRIGGER_MODE_STAND_DOWN = 0
uint8 TRIGGER_MODE_IMMEDIATE = 1
uint8 TRIGGER_MODE_ON_BREAK_BEAM = 2
uint8 TRIGGER_MODE_AT_END = 3

uint8 DRIBBLER_MODE_OFF = 0
uint8 DRIBBLER_MODE_ON = 1
uint8 DRIBBLER_MODE_NEUTRAL = 2

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
