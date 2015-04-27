import single_robot_behavior
import behavior
import robocup
import constants
import main
import math
import enum
import time
import skills._kick


## AngleReceive accepts a receive_point as a parameter and gets setup there to catch the ball
# It transitions to the 'aligned' state once it's there within its error thresholds and is steady
# Set its 'ball_kicked' property to True to tell it to dynamically update its position based on where
# the ball is moving and attempt to catch it.
# It will move to the 'completed' state if it catches the ball, otherwise it will go to 'failed'.
# Kick is a single_robot_behavior, so no need to import both
class AngleReceive(skills._kick._Kick):

    ## max difference between where we should be facing and where we are facing (in radians)
    FaceAngleErrorThreshold = 8 * constants.DegreesToRadians

    ## how much we're allowed to be off in the direction of the pass line
    PositionYErrorThreshold = 0.06

    ## how much we're allowed to be off side-to-side from the pass line
    PositionXErrorThreshold = 0.03

    DribbleSpeed = 70

    ## we have to be going slower than this to be considered 'steady'
    SteadyMaxVel = 0.04
    SteadyMaxAngleVel = 3   # degrees / second

    ## after this amount of time has elapsed after the kick and we haven't received the ball, we failed :(
    ReceiveTimeout = 2



    class State(enum.Enum):
        ## we're aligning with the planned receive point
        aligning = 1

        ## being in this state signals that we're ready for the kicker to kick
        aligned = 2

        ## the ball's been kicked and we're adjusting based on where the ball's moving
        receiving = 3


    def __init__(self):
        super().__init__()

        self.ball_kicked = False
        self._target_pos = None
        self._ball_kick_time = 0

        self._shot_time = 0
        self._shot_occured = None


        for state in AngleReceive.State:
            self.add_state(state, behavior.Behavior.State.running)


        self.add_transition(behavior.Behavior.State.start,
                AngleReceive.State.aligning,
                lambda: True,
                'immediately')

        self.add_transition(AngleReceive.State.aligning,
                AngleReceive.State.aligned,
                lambda: self.errors_below_thresholds() and self.is_steady() and not self.ball_kicked,
                'steady and in position to receive')

        self.add_transition(AngleReceive.State.aligned,
                AngleReceive.State.aligning,
                lambda: (not self.errors_below_thresholds() or not self.is_steady()) and not self.ball_kicked,
                'not in receive position')

        for state in [AngleReceive.State.aligning, AngleReceive.State.aligned]:
            self.add_transition(state,
                    AngleReceive.State.receiving,
                    lambda: self.ball_kicked,
                    'ball kicked')

            self.add_transition(AngleReceive.State.receiving,
                    behavior.Behavior.State.completed,
                    lambda: self._shot_occured,
                    'ball received!')

            self.add_transition(AngleReceive.State.receiving,
                    behavior.Behavior.State.failed,
                    lambda: time.time() - self._ball_kick_time > AngleReceive.ReceiveTimeout,
                    'ball missed :(')



    ## set this to True to let the receiver know that the pass has started and the ball's in motion
    # Default: False
    @property
    def ball_kicked(self):
        return self._ball_kicked
    @ball_kicked.setter
    def ball_kicked(self, value):
        self._ball_kicked = value
        if value:
            self._ball_kick_time = time.time()


    ## The point that the receiver should expect the ball to hit it's mouth
    # Default: None
    @property
    def receive_point(self):
        return self._receive_point
    @receive_point.setter
    def receive_point(self, value):
        self._receive_point = value
        self.recalculate()


    ## returns True if we're facing the right direction and in the right position and steady
    def errors_below_thresholds(self):
        if self.receive_point == None:
            return False

        return (abs(self._angle_error) < AngleReceive.FaceAngleErrorThreshold
                and abs(self._x_error) < AngleReceive.PositionXErrorThreshold
                and abs(self._y_error) < AngleReceive.PositionYErrorThreshold)


    def is_steady(self):
        return (self.robot.vel.mag() < AngleReceive.SteadyMaxVel
                and abs(self.robot.angle_vel) < AngleReceive.SteadyMaxAngleVel)


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
        else:
            # if the ball hasn't been kicked yet, we assume it's going to go through the receive point
            self._pass_line = robocup.Line(ball.pos, self.receive_point)
        self._kick_line = robocup.Line(self.receive_point, self.get_target_point())

        target_angle_rad = (self.get_target_point() - self.robot.pos).angle()
        angle_rad = self.robot.angle
        self._angle_error = target_angle_rad - angle_rad


        if self.ball_kicked:
            receive_before_adjust = self._pass_line.nearest_point(self.robot.pos)
        else:
            receive_before_adjust = self.receive_point

        # Make the receive point be the mouth, rather than the center of the robot.
        # Assumes mouth of robot is at the edge.
        self._target_pos = receive_before_adjust + robocup.Point( \
                constants.Robot.Radius * math.sin(self.robot.angle), \
                constants.Robot.Radius * math.cos(self.robot.angle))

        # Code to provide slipback when receiving the ball
        # pass_line_dir = (self._pass_line.get_pt(1) - self._pass_line.get_pt(0)).normalized()
        # self._target_pos = actual_receive_point + pass_line_dir * constants.Robot.Radius


        # vector pointing down the pass line toward the kicker
        pass_dir = (self._pass_line.get_pt(0) - self._pass_line.get_pt(1)).normalized()

        pos_error = self._target_pos - self.robot.pos
        self._x_error = pos_error.dot(pass_dir.perp_ccw())
        self._y_error = pos_error.dot(pass_dir)


    def get_target_point(self):
        # Gets the point the robot will be facing when receiving the ball
        return constants.Field.TheirGoalCenter

    def on_exit_start(self):
        # reset
        self.ball_kicked = False

    def execute_running(self):
        # make sure teammates don't bump into us
        self.robot.shield_from_teammates(constants.Robot.Radius * 2.0)

        self.recalculate()
        self.robot.face(self.get_target_point())

        if self._pass_line != None:
            main.system_state().draw_line(self._pass_line, constants.Colors.Blue, "Pass")
            main.system_state().draw_line(self._kick_line, constants.Colors.Red, "Shot")
            main.system_state().draw_circle(self._target_pos, 0.03, constants.Colors.Blue, "Pass")



    def execute_aligning(self):
        if self._target_pos != None:
            self.robot.move_to(self._target_pos)

    def on_enter_receiving(self):
        self._shot_time = self.robot.lastKickTime()

    def execute_receiving(self):
        # Kick the ball!
        self.robot.kick(self.kick_power)

        # don't use the move_to() command here, we need more precision, less obstacle avoidance
        pos_error = self._target_pos - self.robot.pos
        vel = pos_error * 3.5
        self.robot.set_world_vel(vel)

        # If the shot took place, end the behavior!
        if self._shot_time != self.robot.lastKickTime():
            self._shot_occured = True

    ## prefer a robot that's already near the receive position
    def role_requirements(self):
        reqs = super().role_requirements()
        if self.receive_point != None:
            reqs.destination_shape = self.receive_point
        return reqs


    def __str__(self):
        desc = super().__str__()
        if self.receive_point != None and self.robot != None:
            desc += "\n    target_pos=" + str(self._target_pos)
            desc += "\n    angle_err=" + str(self._angle_error)
            desc += "\n    x_err=" + str(self._x_error)
            desc += "\n    y_err=" + str(self._y_error)
        return desc
