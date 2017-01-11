import main
import robocup
import math
import constants
import evaluations.ball
# Defensive positioning
# 2 main schools of thought
# Man to man coverage
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
# This is due to win_eval is very slow (From what I have heard)

# Can be estimated most likely with some sort of distance function and delta for the robots in the way

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
# (1) estimate_kick_block_percent(point, our_pos[])
# (2) predict_kick_direction(robot)
# (3) best_defense_positions(ball_pt)
# (4) create_area_defense_zones()
# (5) esteimate_risk_score(pt)

## Estimates the win_eval function so as to not slow down the execution
#
# @param kick_point: Point at which we will kick from
# @param recieve_point: Point at which they are kicking to
# @param blocking_robot_pos[]: list of robots that we should estimate the block for
# @return A percentage chance of block
def estimate_kick_block_percent(kick_point, recieve_point, blocking_robot_pos[]):

    # for each blocking robot pos
    # take angle and distance
    # Move through N different angles
    # Producting a Keneral function for each blocking robot
    # Invert
    # Normalize (Using a square or some sort of thing)
    # Produce percentage from this estimation

    # Array of tuples of all possible blocks
    blocks = []
    kick_direction = (recieve_point - kick_point)
    kick_angle = math.atan2(kick_direction.y, kick_direction.x)

    for pos in blocking_robot_pos:
        block_direction = (pos - kick_point)
        dist = block_direction.mag()
        angle = math.atan2(block_direction.y, block_direction.x) - kick_angle

        if (math.abs(angle) < math.pi / 2 && dist > 0):

        # Kill any that are over pi/2 away
        blocks.extend([(angle, dist)])

    # Standard deviation basically
    half_kick_width = 0.1*math.pi
    num_of_estimates = 10

    total = 0

    for line_offset in range(-half_kick_width, half_kick_width):
        chance = 0
        # How much to decrease with of kernel per distance away
        # Inf @ 0 to 0 @ pi / 2
        distance_scale = 1 / math.abs(math.tan(line_num))

        for pos  in blocks:
            # Produce a number between -1 and 1
            # Find u based off of line_offset
            # Scale output then based on distance
            u = line_offset * distance_scale
            chance += max((35/32)*pow((1-pow(u,2)), 3), 0)

    # Invert
    # Normalize
    # Produce percentage

    pass
    # TODO: Figure out the math for this
    # Most likely project the blocking robots onto the kick_line
    # Then use a delta distance and the distance down the kick_line to scale the robot shadow
    # to the recieve_point
    # (Each robot shadow will have a width and center)
    # Unless there is an easy way to merge them
    # Just down N points and see if they are contained in an area

## Predicts the impending kick direction based on the angle of the robot and the
#  angle of approach
#
# @param robot: The robot which we want to estimate
# @return Angle of most likely kick
def predict_kick_direction(robot):
    angle = robot.angle
    pos = robot.pos
    angle_vel = robot.angle_vel

    # Find interception between robot and ball
    # Use distance from bot to ball to predict time it takes to intercept
    # Assumes their robot has about the same physical characteristics as our robots
    # TODO: Predict this better time_to_ball assumes zero veloctiy
    inst_ball_time = time_to_ball(robot)
    future_ball_pos = predict(main.ball().pos, main.ball().vel, inst_ball_time)
    direction = (future_ball_pos - pos).normalized()

    robot_angle_predict = angle + angle_vel*inst_ball_time
    # TODO: Make sure arctan produces the right rotation
    ball_angle_predict = math.degrees(math.atan2(direction.y, direction.x))

    c = 0.7
    return c * robot_angle_predict + (1-c) * ball_angle_predict

    # TODO: Figure out the math for this
    # Take in the position / velocity / accel of the translation of the robot
    # Do a quick prediction based on interception in the furture
    # Estimate time of interception
    # Take in the angle  / velocity / accel of the yaw of the robot
    # Do a quick prediction in X seconds in the future based on time of interception
    # (To allow our robot to move into place if we are off)
    #
    # Throw into a complementary filter as one will most likely be more accurate than
    # the other one

## Creates a zone that may cause a risk in the future
#  Based off the...
#   Risk score in that position
#   Adviailbity of opponent robots to reach that point
#   Space in that area
#
# @return List of the top 2 rectangle zones and their scores
def create_area_defense_zones():
    pass
    # Find out areas with a strong pass chance (Estimate_risk_score)
    # Weight space significantly higher, decrease areas away from center
    # Increase weight for areas that other robots can move into easily
    # (Create a cone away from the goal with a distance weight)
    #
    # Merge areas that are further away from the ball along the same angle

## Estimates how dangerous an enemy robot can be at a certain point
#
# @return Risk score at that point
def estimate_risk_score(point):
    pass
    # Merge of a few different scores
    # Openness of pass / shot
    # Time it will take before they are accitvated within the play (exponetial)

## Decides where to move the three robots
#
# @return List of the 3 defensive positions
def find_defense_positions():
    pass
    # The bread and butter of this whole thing
    # Returns the list of best defense positions (TOP N Amounts)

    # Take in a list of area defense zones (create_area_defense_zones)
    # Produce a list of the highest threat robots (estimate_risk_score(robot_pos))
    
    # Grab the closest robot to the ball
    # Approach the ball slow-ish but be very sensitive to side to side movement
    # Block goal on approach

    # Move second robot to block pass / second robot shot
    # TODO: Decide whether it is better to blcok pass or block shot
    # Shot is lower risk to block
    # Pass is higher reward to block
    # Can base on score as well previous pass block success rates
    # Can also position to be able to get both
    # Mirror movements of other robot

    # Move third to defend the area defense
    # or defend the pass / shot of the last robot
    # Best would be to find best position to defend both