import sys
sys.path.insert(1, "../../stp")
import rc
import math
import numpy as np
from typing import Optional

class Ball:
    
    def distance(loc1, loc2):
        #return (abs())
        return 0.0
    """manifestation of a ball"""

    '''
    def is_moving_towards_our_goal() -> bool:
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
    '''

    def intersect(s1, s2, b1, b2):
    	''' do the lines s and b intersect?'''
        print(s1[0], s1[1])
        print(s2[0], s2[1])
        print(b1[0], b1[1])
        print(b2[0], b2[1])
        
        #x check
        xmin = min(s1[0], s2[0])
        xmax = max(s1[0], s2[0])
        x = ((xmin <= b1[0] <= xmax) or (xmin <= b2[0] <= xmax))
        #y check
        ymin = min(s1[1], s2[1])
        ymax = max(s1[1], s2[1])
        y = ((ymin <= b1[1] <= ymax) or (ymin <= b2[1] <= ymax))
        return (x and y)

    def is_moving_backward(ball, field):

        '''
        Is the ball moving towards the opponent's goal (forward)
        or towards our goal (backward)?
        '''
        
        vel = np.asarray(ball.vel)
        mag = math.sqrt(vel[0]**2 + vel[1]**2)
        nvel = vel / mag
        if mag > 0.18: #vision noise should be thresholded out
            if vel.dot([0, -1]) > 0:	
                ball_path = [ball.pos, (
                ball.pos + nvel)]
                print(ball_path[0], ball_path[1])
                fudge_factor = 0.15  # TODO: this could be tuned better
                GoalSegment = [[field.goal_width_m / 2.0 + fudge_factor,
                            0],
                    [-field. goal_width_m/2.0 - fudge_factor,
                            0]]   
                #print(GoalSegment[0], GoalSegment[1])
                #pt = ball_path.segment_intersection(WiderGoalSegment)
                pt = intersect(GoalSegment[0], GoalSegment[1], ball_path[0], ball_path[1])
                return pt
        return False

    def is_in_our_goalie_zone(ball, field):
        if ball != None:
            return False
            #return constants.Field.OurGoalZoneShape.contains_point(main.ball().pos)
        else:
            return False

    def we_are_closer(worldstate):
       
        ball = rc.ball()
        our_robots = rc.our_robots()
        their_robots = rc.their_robots()
        '''
        return min([(ball.pos - rob.pos).mag()
                for rob in main.system_state().their_robots]) > min(
                    [(main.ball().pos - rob.pos).mag()
                     for rob in main.system_state().our_robots])
        '''
        
    	return False

    def opponent_is_much_closer(worldstate):
        ball = rc.ball()
        our_robots = rc.our_robots()
        their_robots = rc.their_robots()
        '''
        return min([(ball.pos - rob.pos).mag()
                for rob in main.system_state().their_robots]) * 3 < min(
                    [(main.ball().pos - rob.pos).mag()
                     for rob in main.system_state().our_robots])
        '''
    	return False

    def moving_slow(ball):
        vel = sqrt(ball.vel()[0]**2 + ball.vel()[1]**2)
        #return vel <= constants.Evaluation.SlowThreshold
    	return False

    FrictionCoefficient = 0.04148
    GravitationalCoefficient = 9.81  # in m/s^2

    def predict_stop_time(ball):
        #return main.ball().predict_seconds_to_stop()
    	return 0

    def predict_stop(ball):
        #return ball.predict_pos(ball.predict_seconds_to_stop())
    	return 0

    def rev_predict(dist, ball):
        """predict how much time it will take the ball to travel the given distance"""
        #return main.ball().estimate_seconds_to_dist(dist)
        vel = ball.vel()
        v = sqrt(vel[0]**2 + vel[1]**2)
        
        return 0.0

    def opponent_with_ball(ball):
        max_dist = float('inf')
        '''
        closest_bot, closest_dist = None, float("inf")
    for bot in main.their_robots():
        if bot.visible:
            dist = (bot.pos - main.ball().pos).mag()
            if dist < closest_dist:
                closest_bot, closest_dist = bot, dist

    if closest_bot is None:
        return None
    else:
        if robot_has_ball(closest_bot):
            return closest_bot
        else:
            return None
        '''
    	return None

    def our_robot_with_ball(ball):
        max_dist = float('inf')
        '''
        closest_bot, closest_dist = None, float("inf")
    for bot in main.our_robots():
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
        '''
        return None

    def robot_has_ball(ball):
        mouth_half_angle = math.pi / 12
        
        '''
    max_dist_from_mouth = 1.13 * (
        constants.Robot.Radius + constants.Ball.Radius)

    # Create triangle between bot pos and two points of the mouth
    A = robot.pos
    B = A + robocup.Point(
        max_dist_from_mouth * math.cos(robot.angle - mouth_half_angle),
        max_dist_from_mouth * math.sin(robot.angle - mouth_half_angle))
    C = A + robocup.Point(
        max_dist_from_mouth * math.cos(robot.angle + mouth_half_angle),
        max_dist_from_mouth * math.sin(robot.angle + mouth_half_angle))
    D = main.ball().pos

    # Barycentric coordinates to solve whether the ball is in that triangle
    area = 0.5 * (-B.y * C.x + A.y * (-B.x + C.x) + A.x *
                  (B.y - C.y) + B.x * C.y)
    s = 1 / (2 * area) * (A.y * C.x - A.x * C.y + (C.y - A.y) * D.x +
                          (A.x - C.x) * D.y)
    t = 1 / (2 * area) * (A.x * B.y - A.y * B.x + (A.y - B.y) * D.x +
                          (B.x - A.x) * D.y)

    # Due to the new camera configuration in the 2019 year,
    # the ball dissapears consistently when we go to capture a ball near the
    # edge of the field. This causes the ball to "appear" inside the robot
    # so we should assume that if the ball is inside, we probably have
    # the ball
    ball_inside_robot = (robot.pos - main.ball().pos).mag() < \
                        constants.Robot.Radius + constants.Ball.Radius

    return (s > 0 and t > 0 and (1 - s - t) > 0) or ball_inside_robot
        '''
    	return None

    def time_to_ball(ball, robot):
        
        '''
        max_vel = robocup.MotionConstraints().max_speed
    max_accel = robocup.MotionConstraints().max_accel
    delay = .1  # TODO: tune this better
    rpos = robot.pos
    bpos = main.ball().pos
    # calculate time for self to reach ball using max_vel + a slight delay for capture
    dist_to_ball = robot.pos.dist_to(main.ball().pos)
    return (dist_to_ball / max_vel) + delay
        '''
    	return 0

    ball = rc.Ball([0,4.5],[0, -1], True)
    field = rc.Field(9, 6, 1, 1, 2, 0.5, 0.5, 1, 2, 1.5, 3, 1, 8, 10)

    forward = is_moving_backward(ball, field)
    if forward:
    	print("Ball is moving towards our goal.")
    else:
    	print("Ball is not moving towards our goal.")
	
'''
import main
import robocup
import constants
import math
from typing import Optional


def is_moving_towards_our_goal() -> bool:
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


def is_in_our_goalie_zone() -> bool:
    if main.ball() != None:
        return constants.Field.OurGoalZoneShape.contains_point(main.ball().pos)
    else:
        return False


# TODO use for situation analysis
def we_are_closer() -> bool:
    return min([(main.ball().pos - rob.pos).mag()
                for rob in main.system_state().their_robots]) > min(
                    [(main.ball().pos - rob.pos).mag()
                     for rob in main.system_state().our_robots])


# TODO use for situation analysis
def opponent_is_much_closer() -> bool:
    return min([(main.ball().pos - rob.pos).mag()
                for rob in main.system_state().their_robots]) * 3 < min(
                    [(main.ball().pos - rob.pos).mag()
                     for rob in main.system_state().our_robots])


def moving_slow() -> bool:
    return main.ball().vel.mag() <= constants.Evaluation.SlowThreshold


FrictionCoefficient = 0.04148
GravitationalCoefficient = 9.81  # in m/s^2


def predict_stop_time() -> float:
    return main.ball().predict_seconds_to_stop()


def predict_stop() -> float:
    return main.ball().predict_pos(main.ball().predict_seconds_to_stop())


def rev_predict(dist) -> float:
    """predict how much time it will take the ball to travel the given distance"""
    return main.ball().estimate_seconds_to_dist(dist)


# returns a Robot or None indicating which opponent has the ball
def opponent_with_ball() -> Optional[robocup.OpponentRobot]:
    closest_bot, closest_dist = None, float("inf")
    for bot in main.their_robots():
        if bot.visible:
            dist = (bot.pos - main.ball().pos).mag()
            if dist < closest_dist:
                closest_bot, closest_dist = bot, dist

    if closest_bot is None:
        return None
    else:
        if robot_has_ball(closest_bot):
            return closest_bot
        else:
            return None


## If our robot has the ball, then returns that robot. Otherwise None
#
# @return Robot: a robot or None
def our_robot_with_ball() -> Optional[robocup.OurRobot]:
    closest_bot, closest_dist = None, float("inf")
    for bot in main.our_robots():
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
def robot_has_ball(robot: robocup.Robot) -> bool:
    mouth_half_angle = 15 * math.pi / 180  # Angle from front
    max_dist_from_mouth = 1.13 * (
        constants.Robot.Radius + constants.Ball.Radius)

    # Create triangle between bot pos and two points of the mouth
    A = robot.pos
    B = A + robocup.Point(
        max_dist_from_mouth * math.cos(robot.angle - mouth_half_angle),
        max_dist_from_mouth * math.sin(robot.angle - mouth_half_angle))
    C = A + robocup.Point(
        max_dist_from_mouth * math.cos(robot.angle + mouth_half_angle),
        max_dist_from_mouth * math.sin(robot.angle + mouth_half_angle))
    D = main.ball().pos

    # Barycentric coordinates to solve whether the ball is in that triangle
    area = 0.5 * (-B.y * C.x + A.y * (-B.x + C.x) + A.x *
                  (B.y - C.y) + B.x * C.y)
    s = 1 / (2 * area) * (A.y * C.x - A.x * C.y + (C.y - A.y) * D.x +
                          (A.x - C.x) * D.y)
    t = 1 / (2 * area) * (A.x * B.y - A.y * B.x + (A.y - B.y) * D.x +
                          (B.x - A.x) * D.y)

    # Due to the new camera configuration in the 2019 year,
    # the ball disappears consistently when we go to capture a ball near the
    # edge of the field. This causes the ball to "appear" inside the robot
    # so we should assume that if the ball is inside, we probably have
    # the ball
    ball_inside_robot = (robot.pos - main.ball().pos).mag() < \
                        constants.Robot.Radius + constants.Ball.Radius

    return (s > 0 and t > 0 and (1 - s - t) > 0) or ball_inside_robot


def time_to_ball(robot: robocup.Robot) -> float:
    max_vel = robocup.MotionConstraints.MaxRobotSpeed.value
    max_accel = robocup.MotionConstraints.MaxRobotAccel.value
    delay = .1  # TODO: tune this better
    rpos = robot.pos
    bpos = main.ball().pos
    # calculate time for self to reach ball using max_vel + a slight delay for capture
    dist_to_ball = robot.pos.dist_to(main.ball().pos)
    return (dist_to_ball / max_vel) + delay
'''
