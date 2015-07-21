import single_robot_behavior
import behavior
import robocup
import constants
import main
import math
import enum
import time
import skills._kick
import skills.pass_receive


## AngleReceive accepts a receive_point as a parameter and gets setup there to catch the ball
# It transitions to the 'aligned' state once it's there within its error thresholds and is steady
# Set its 'ball_kicked' property to True to tell it to dynamically update its position based on where
# the ball is moving and attempt to catch it.
# It will move to the 'completed' state if it catches the ball, otherwise it will go to 'failed'.
# Kick is a single_robot_behavior, so no need to import both
class AngleReceive(skills.pass_receive.PassReceive):

    def __init__(self):
        self.kick_power = 1
        super().__init__()

    ## Returns an adjusted angle with account for ball speed
    #
    # First finds the rejection, which is the X component of the ball's velocity in the reference
    # frame of the robot, with the mouth facing the y axis. Then we calculate the angle required to
    # offset this rejection angle (if possible).
    def adjust_angle(self, target_angle, ball_angle = None, ball_speed = None):
        ball = main.ball()

        if ball_angle == None:
            ball_angle = (ball.vel).angle()

        if ball_speed == None:
            ball_speed = ball.vel.mag()

        angle_diff = target_angle - ball_angle
        rejection = math.sin(angle_diff) * ball_speed

        # The min/max is to bound the value by -1 and 1.
        adjust = math.asin(min(1, max(-1, rejection / constants.Robot.MaxKickSpeed)))
        return adjust + target_angle

    # calculates:
    # self._pass_line - the line from the ball along where we think we're going
    # self._target_pos - where the bot should be
    # self._angle_error - difference in where we're facing and where we want to face (in radians)
    # self._x_error
    # self._y_error
    def recalculate(self):
        # can't do squat if we don't know what we're supposed to do
        if self.receive_point == None or self.robot == None:
            return


        ball = main.ball()

        if self.ball_kicked:
            # when the ball's in motion, the line is based on the ball's velocity
            self._pass_line = robocup.Line(ball.pos, ball.pos + ball.vel*10)

            # After kicking, apply angle calculations
            target_angle_rad = self.adjust_angle((self.get_target_point() - self.robot.pos).angle())
            # Removes angle adjustment
            # target_angle_rad = (self.get_target_point() - self.robot.pos).angle()

            self._kick_line = robocup.Line(self.robot.pos, robocup.Point(self.robot.pos.x + math.cos(self.robot.angle) * 10, self.robot.pos.y + math.sin(self.robot.angle) * 10))
        else:
            # if the ball hasn't been kicked yet, we assume it's going to go through the receive point
            self._pass_line = robocup.Line(ball.pos, self.receive_point)
            # Assume ball is kicked at max speed and is coming from the ball point to the location of our robot. Then average this with the target angle.
            target_angle_rad = self.adjust_angle((self.get_target_point() - self.robot.pos).angle(), (self.robot.pos - main.ball().pos).angle(), constants.Robot.MaxKickSpeed)
            # TODO make this faster by caching the .angle() part
            target_angle_rad = (target_angle_rad + (self.get_target_point() - self.robot.pos).angle()) / 2

            self._kick_line = robocup.Line(self.receive_point, self.get_target_point())

        self._angle_facing =  target_angle_rad
        angle_rad = self.robot.angle
        self._angle_error = target_angle_rad - angle_rad

        if self.ball_kicked:
            receive_before_adjust = self._pass_line.nearest_point(self.robot.pos)
        else:
            receive_before_adjust = self.receive_point

        # Make the receive point be the mouth, rather than the center of the robot.
        # Assumes mouth of robot is at the edge.
        self._target_pos = receive_before_adjust - robocup.Point(
                constants.Robot.Radius * math.cos(self.robot.angle),
                constants.Robot.Radius * math.sin(self.robot.angle))

        # Code to provide slipback when receiving the ball
        # pass_line_dir = (self._pass_line.get_pt(1) - self._pass_line.get_pt(0)).normalized()
        # self._target_pos = actual_receive_point + pass_line_dir * constants.Robot.Radius


        # vector pointing down the pass line toward the kicker
        pass_dir = (self._pass_line.get_pt(0) - self._pass_line.get_pt(1)).normalized()

        pos_error = self._target_pos - self.robot.pos
        self._x_error = self._target_pos.x - self.robot.pos.x
        self._y_error = self._target_pos.y - self.robot.pos.y


    def get_target_point(self):
        # Gets the point the robot will be facing when receiving the ball
        return constants.Field.TheirGoalSegment.center()

    def execute_running(self):
        super().execute_running()

        self.robot.face(self.robot.pos
                + robocup.Point(math.cos(self._angle_facing), math.sin(self._angle_facing)))
        if self._kick_line != None:
            main.system_state().draw_line(self._kick_line, constants.Colors.Red, "Shot")

    def execute_receiving(self):
        super().execute_receiving()

        # Kick the ball!
        self.robot.kick(self.kick_power)

