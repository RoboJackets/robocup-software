import main
import robocup
import constants
import math


def is_moving_towards_our_goal():
    # see if the ball is moving much
    if main.ball().vel.mag() > 0.18:  # Tuned based on vision noise
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


def predict_stop_time():
    return main.ball().predict_seconds_to_stop()


def predict_stop():
    return main.ball().predict_pos(main.ball().predict_seconds_to_stop())

def rev_predict(dist):
    """predict how much time it will take the ball to travel the given distance"""
    return main.ball().estimate_seconds_to_dist(dist)

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
    mouth_half_angle = 15*math.pi/180 # Angle from front
    max_dist_from_mouth = 1.13 * (
        constants.Robot.Radius + constants.Ball.Radius)

    # Create triangle between bot pos and two points of the mouth
    A = robot.pos
    B = A + robocup.Point(
            max_dist_from_mouth*math.cos(robot.angle - mouth_half_angle),
            max_dist_from_mouth*math.sin(robot.angle - mouth_half_angle))
    C = A + robocup.Point(
            max_dist_from_mouth*math.cos(robot.angle + mouth_half_angle),
            max_dist_from_mouth*math.sin(robot.angle + mouth_half_angle))
    D = main.ball().pos

    # Barycentric coordinates to solve whether the ball is in that triangle
    area = 0.5*(-B.y*C.x + A.y*(-B.x+C.x) + A.x*(B.y - C.y) + B.x*C.y)
    s = 1/(2*area) * (A.y*C.x - A.x*C.y + (C.y - A.y)*D.x + (A.x - C.x)*D.y)
    t = 1/(2*area) * (A.x*B.y - A.y*B.x + (A.y - B.y)*D.x + (B.x - A.x)*D.y)

    # Due to the new camera configuration in the 2019 year,
    # the ball dissapears consistently when we go to capture a ball near the
    # edge of the field. This causes the ball to "appear" inside the robot
    # so we should assume that if the ball is inside, we probably have
    # the ball
    ball_inside_robot = (robot.pos - main.ball().pos).mag() < \
                        constants.Robot.Radius + constants.Ball.Radius

    return (s > 0 and t > 0 and (1 - s - t) > 0) or ball_inside_robot


def time_to_ball(robot):
    max_vel = robocup.MotionConstraints.MaxRobotSpeed.value
    max_accel = robocup.MotionConstraints.MaxRobotAccel.value
    delay = .1  #TODO: tune this better
    rpos = robot.pos
    bpos = main.ball().pos
    #calculate time for self to reach ball using max_vel + a slight delay for capture
    dist_to_ball = robot.pos.dist_to(main.ball().pos)
    return (dist_to_ball / max_vel) + delay
