# The team's name. Empty string if operator has not typed anything.
string name ""

# The number of goals scored by the team during normal play and overtime.
uint32 score 0

# The number of red cards issued to the team since the beginning of the game.
uint32 red_cards 0

# The amount of time (in microseconds) left on each yellow card issued to the team.
# If no yellow cards are issued, this array has no elements.
# Otherwise, times are ordered from smallest to largest.
uint32[] yellow_card_times

# The total number of yellow cards ever issued to the team.
uint32 yellow_cards 0

# The number of timeouts this team can still call. If in a timeout right now, that timeout is excluded.
uint32 timeouts_left 0

# The number of microseconds of timeout this team can use.
uint32 timeout_time 0

# The pattern number of this team's goalie.
uint32 goalie 0
