GameStage stage
GameCommand command
GameRestart restart

# True if our team can kick the ball during a restart.
bool our_restart

int32 our_score
int32 their_score

# Time remaining in the current period.
builtin_interfaces/Duration stage_time_left

TeamInfo our_info
TeamInfo their_info

# Bool representing if we are on the blue team.
bool blue_team

rj_geometry_msgs/Point ball_placement_point

# Raw stage
# ...

# Raw command
# ...