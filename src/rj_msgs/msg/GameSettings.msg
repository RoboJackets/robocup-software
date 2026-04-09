# Whether or not we're in simulation.
bool simulation

# Requests. These can be overridden by the referee if it's enabled
bool request_blue_team
int32 request_goalie_id

# Defend the plus-x direction in vision
bool defend_plus_x

bool use_our_half
bool use_their_half

bool paused

# Motion test robot selection. -1 means "None".
int32 motion_test_robot_id

# Motion test selection enum.
# 0=NONE, 1=YELLOW_STRAIGHT, 2=BLUE_STRAIGHT, 3=BORDER_FIELD, 4=DEFAULT_FIELD.
int32 motion_test_type
