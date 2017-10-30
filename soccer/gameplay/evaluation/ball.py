import main
import robocup
import constants
import math


def is_moving_towards_our_goal():
    # see if the ball is moving much
    if main.ball().vel.mag() > 0.1:
        # see if it's moving somewhat towards our goal
        if main.ball().vel.dot(robocup.Point(0, -1)) > 0:
            ball_path = robocup.Line(main.ball().pos, (
                main.ball().pos + main.ball().vel.normalized()))

            fudge_factor = 0.15  # TODO: this could be tuned better
            WiderGoalSegment = robocup.Segment(
                robocup.Point(constants.Field.GoalWidth / 2.0 + fudge_factor,
                              0),
                robocup.Point(-constants.Field.GoalWidth / 2.0 - fudge_factor,
                              0))

            pt = ball_path.segment_intersection(WiderGoalSegment)
            return pt != None

    return False


def is_in_our_goalie_zone():
    if main.ball() != None:
        return constants.Field.OurGoalZoneShape.contains_point(main.ball().pos)
    else:
        return False


FrictionCoefficient = 0.04148
GravitationalCoefficient = 9.81  # in m/s^2


# The ball's motion follows the equation X(t) = X_i + V_i*t - 0.5*(c*g)*t^2
def predict(X_i, V_i, t):
    return X_i + (V_i * t) - (V_i.normalized() * 0.5 * FrictionCoefficient *
                              GravitationalCoefficient * t**2)


def predict_stop_time(start_speed):
    return (start_speed / (FrictionCoefficient * GravitationalCoefficient))


def predict_stop(X_i, V_i):
    return predict(X_i, V_i, predict_stop_time(V_i.mag()))


def rev_predict(V_i, dist):
    """predict how much time it will take the ball to travel the given distance"""

    # use the quadratic formula (a^2x + bx + c = 0 -> (-b +/- sqrt(b^2-4ac)) / 2a)
    a = -0.5 * FrictionCoefficient * GravitationalCoefficient
    b = V_i.mag()
    c = -dist

    # we ignore the second solution because it doesn't make sense in this context
    b4ac = b**2 - 4 * a * c
    if b4ac > 0:
        return (-b + math.sqrt(b4ac)) / 2 * a
    else:
        return float("inf")


# returns a Robot or None indicating which opponent has the ball
def opponent_with_ball():
    closest_bot, closest_dist = None, float("inf")
    for bot in main.their_robots():
        if bot.visible:
            dist = (bot.pos - main.ball().pos).mag()
            if dist < closest_dist:
                closest_bot, closest_dist = bot, dist

    if closest_bot == None:
        return None
    else:
        if robot_has_ball(closest_bot):
            return closest_bot
        else:
            return None


# based on face angle and distance, determines if the robot has the ball
def robot_has_ball(robot):
    def angle_btw_three_pts(a, b, vertex):
        return (a-vertex).angle_between(b-vertex)

    angle = robot.angle
    theta = angle_btw_three_pts(robot.pos + robocup.Point(
        math.cos(angle), math.sin(angle)), main.ball().pos, robot.pos)
    max_radius = constants.Robot.Radius * (2.0 + 2.0 * math.cos(theta))

    return robot.pos.near_point(main.ball().pos, max_radius)


def time_to_ball(robot):
    max_vel = robocup.MotionConstraints.MaxRobotSpeed.value
    max_accel = robocup.MotionConstraints.MaxRobotAccel.value
    delay = .1  #TODO: tune this better
    rpos = robot.pos
    bpos = main.ball().pos
    #calculate time for self to reach ball using max_vel + a slight delay for capture
    dist_to_ball = robot.pos.dist_to(main.ball().pos)
    return (dist_to_ball / max_vel) + delay
