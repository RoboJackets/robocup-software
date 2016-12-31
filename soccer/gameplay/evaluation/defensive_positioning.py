# Defensive positioning
# 2 main schools of thought
# Man 2 man coverage
# Zone coverage

# A mix of two will be the best
# We need to figure out good algorithms to find a position for each robot

# Zone defense works on keep robots somewhat spread for the incoming cross pass
# This also helps with a counter attack as one robot is free on the side of the field

# With defense, you want to push the opponents outside and cover the middle and any cross passes
# In robocup, covering crosses is a little harder due to how quickly the ball moves in comparison
# to the robots themselves.
# With that, a stronger angle defense is needed.
# Angle defense is needed to close distance to cover a wider angle
# Figure out the mathimatical relationship between distance and angle coverage
# I would imagine it is somewhat of a power / exp situation

# (1) Next, figure out a quick way to estimate the shot chance of a robot on the opponent side
# This is due to win_eval is very slow (From what I have seen)

# Can be estimated most likely with some sort of distance function and delta for the closest robot

# Zones will need to be figured out physically until we can test multiple different ones against each other
# One basic system would be to just do a third of the field on either side of the center line

# (2) Defense wise, the robots can only pass form their mouth
# we can get that angle as well as the angle of their approach to do some early estimations
# of pass / shot direction
# A complementary filter on those two shouldn't be too hard
# Mostly likely weight towards the predicted mouth angle a little bit more (use at the very least velocity, hopefully acceleration too)

# (3) Next we need to get a list of robot positions that we should move into
# This would require figureing out their best pass / shot posiitions
# Move to block both their current positions
# As well stay in positions to block future pass positions
# Use space_coeff to find large pockets of open space and good shots from that positions
# Only one need to defend there

# (4) Area  defense will be used for zones further away from the ball
# We ened to cover as many zones as possible further away so finding a good area for each robot away from the action is needed
# To do this, we need to find where there is a significant amount of space
# What the best angle is from the ball
# And any nearby robots that we will need to move into defense for
# We can find a good position to pass to (using eight furthest time if multiple points are needed to be covered
# or just the center of the group if they are somewhat close)
# We also need to find the closest bot that is incoming that we need to cover
# Decide if covering the pass or the shot is better
# Minimize risk to find the best
# (5) Risk can be considered the closeness of another robot inc to the shot / pass location and their possible leathality
# in that position
# Risk must take into account the direction of attack onto the ball
# As well as the defense in those positions

###############################
## Most likely needed functions
###############################
#
# 
# (1) estimate_kick_block_percent(point, point)
# (2) predict_kick_direction(robot)
# (3) best_defense_positions(ball_pt)
# (4) create_area_defense_zones()
# (5) esteimate_risk_score(pt)