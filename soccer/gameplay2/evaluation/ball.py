import main
import robocup
import constants
import math


def is_moving_towards_our_goal():
    # see if the ball is moving much
    if main.ball().vel.mag() > 0.04:
        ctr2goal = robocup.Point(0, constants.Field.Length / 2.0).normalized()

        # see if it's moving somewhat towards our goal
        if main.ball().vel.dot(ctr2goal) > 0:
            ball_path = robocup.Segment(main.ball().pos, (main.ball().pos + main.ball().vel.normalized()))
            pt = ball_path.line_intersection(constants.Field.OurGoalSegment)
            return pt != None and abs(pt.x) < constants.Field.GoalWidth / 2.0

    return False


def is_in_our_goalie_zone():
    if main.ball() != None:
        return constants.Field.OurGoalZoneShape.contains_point(main.ball().pos)
    else:
        return False


FrictionCoefficient = 0.04148
GravitationalCoefficient = 9.81 # in m/s^2

# The ball's motion follows the equation X(t) = X_i + V_i*t - 0.5*(c*g)*t^2
def predict(X_i, V_i, t):
    return X_i + (V_i * t) - (0.5 * FrictionCoefficient * GravitationalCoefficient * t**2)


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
    def angle_btw_three_pts(a, b, vertex):
        VA = math.sqrt( (vertex.x - a.x)**2 + (vertex.y - a.y)**2 )
        VB = math.sqrt( (vertex.x - b.x)**2 + (vertex.y - b.y)**2 )
        AB = math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 )
        return math.acos((VA*VA + VB*VB - AB*AB)/(2*VA*VB))

    closest_bot = None
    closest_dist = float("inf")
    for bot in main.their_robots():
        if bot.visible:
            dist = (bot.pos - main.ball().pos).mag()
            if dist < closest_dist:
                closest_bot, closest_dist = bot, dist

    if closest_bot == None:
        return None

    angle = closest_bot.angle * constants.DegreesToRadians
    theta = angle_btw_three_pts(closest_bot.pos + robocup.Point(math.cos(angle), math.sin(angle)),
                                main.ball().pos,
                                closest_bot.pos)
    max_radius = constants.Robot.Radius * (2.0 + 2.0*math.cos(theta))

    if closest_bot.pos.near_point(main.ball().pos, max_radius):
        return closest_bot
    else:
        return None
