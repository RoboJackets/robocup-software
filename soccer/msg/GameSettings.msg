# Whether or not we're in simulation.
bool simulation

# Whether external referee is allowed.
# If this is set to true but no external referee is connected,
# none will be used and the control panel can be used as always.
bool use_external_referee

# Requests. These can be overridden by the referee if it's enabled
bool request_blue_team
int32 request_goalie_id

# Defend the plus-x direction in vision
bool defend_plus_x

bool use_our_half
bool use_their_half

bool paused
