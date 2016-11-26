import single_robot_composite_behavior
import behavior
import robocup
import constants
import main
import enum
import math
import time
import role_assignment
import skills


## PassReceive accepts a receive_point as a parameter and gets setup there to catch the ball
# It transitions to the 'aligned' state once it's there within its error thresholds and is steady
# Set its 'ball_kicked' property to True to tell it to dynamically update its position based on where
# the ball is moving and attempt to catch it.
# It will move to the 'completed' state if it catches the ball, otherwise it will go to 'failed'.
class PassReceive(single_robot_composite_behavior.SingleRobotCompositeBehavior):

    ## max difference between where we should be facing and where we are facing (in radians)
    FaceAngleErrorThreshold = 8 * constants.DegreesToRadians

    MarginAngle = math.pi / 18
    StabilizationFrames = 3
    DesperateTimeout = 5

    class State(enum.Enum):
        # Not close enough to recieve pass, closing distance
        closing = 1

        # Ready to collect ball (Still moving into recieve point)
        in_range = 2

        # The ball has been kicked and we are collecting it
        collecting = 3

    def __init__(self, captureFunction=(lambda: skills.capture.Capture())):
        super().__init__(continuous=False)

        self.ball_kicked = False
        self._target_pos = None
        self._receive_point = None
        self._ball_kick_time = 0
        self.kicked_from = None
        self.kicked_vel = None
        self.stable_frame = 0
        self.kicked_time = 0
        self.captureFunction = captureFunction

        for state in PassReceive.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            PassReceive.State.closing, lambda: True,
                            'immediately')

        self.add_transition(
            PassReceive.State.closing, PassReceive.State.in_range,
            lambda: self.errors_below_thresholds() and not self.ball_kicked,
            'in position to receive')

        self.add_transition(
            PassReceive.State.in_range, PassReceive.State.closing,
            lambda: (not self.errors_below_thresholds() and not self.ball_kicked,
            'not in receive position')

        for state in [PassReceive.State.in_range, PassReceive.State.closing]:
            self.add_transition(state, PassReceive.State.collecting,
                                lambda: self.ball_kicked, 'ball kicked')

        self.add_transition(PassReceive.State.collecting,
                            behavior.Behavior.State.completed,
                            lambda: self.robot.has_ball(), 'ball received!')

        self.add_transition(
            PassReceive.State.receiving, behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.failed or self.check_failure() or time.time() - self.kicked_time > PassReceive.DesperateTimeout,
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

    # Return true if the robot is close enough to collect the ball
    def errors_below_thresholds(self):
        if self.receive_point == None:
            return False

        # predict time for robot to reach point
        # Predict time for ball to be fully passed
        return False

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
            self._pass_line = robocup.Line(ball.pos, ball.pos + ball.vel * 10)
        else:
            # if the ball hasn't been kicked yet, we assume it's going to go through the receive point
            self._pass_line = robocup.Line(ball.pos, self.receive_point)

        target_angle_rad = (ball.pos - self.robot.pos).angle()
        angle_rad = self.robot.angle
        self._angle_error = target_angle_rad - angle_rad

        # TODO: See if this needs to be change, this wont work very well with the motion
        if self.ball_kicked:
            actual_receive_point = self._pass_line.nearest_point(
                self.robot.pos)
        else:
            actual_receive_point = self.receive_point

        pass_line_dir = (
            self._pass_line.get_pt(1) - self._pass_line.get_pt(0)).normalized()
        self._target_pos = actual_receive_point + pass_line_dir * constants.Robot.Radius

        # vector pointing down the pass line toward the kicker
        pass_dir = (
            self._pass_line.get_pt(0) - self._pass_line.get_pt(1)).normalized()

        pos_error = self._target_pos - self.robot.pos
        self._x_error = pos_error.dot(pass_dir.perp_ccw())
        self._y_error = pos_error.dot(pass_dir)

    def on_exit_start(self):
        # reset
        self.ball_kicked = False

    def execute_running(self):
        self.recalculate()
        self.robot.face(main.ball().pos)

        if self._pass_line != None:
            main.system_state().draw_line(self._pass_line,
                                          constants.Colors.Blue, "Pass")
            main.system_state().draw_circle(self._target_pos, 0.03,
                                            constants.Colors.Blue, "Pass")

    def execute_closing(self):
        if self._target_pos != None:
            self.robot.move_to(self._target_pos)

    def reset_correct_location(self):
        # Extrapolate center of robot location from kick velocity
        self.kicked_from = main.ball().pos  #- (main.ball().vel / main.ball().vel.mag()) * constants.Robot.Radius * 4
        self.kicked_vel = main.ball().vel

    #TODO: Create a better capture function (Decide if time to collect or position should be minimized)
    def on_enter_collecting(self):
        capture = self.captureFunction()
        self.add_subbehavior(capture, 'capture', required=True)

        self.reset_correct_location()
        self.kicked_time = time.time()

    def on_exit_collecting(self):
        self.remove_subbehavior('capture')

    ## Create a good_area, that determines where a good pass should be,
    # return true if the ball has exited that area.
    #
    # Run test_coordinated_pass for an example of this.
    def check_failure(self):
        # We wait about 3 frames before freezing the velocity and position of the ball
        # as it can be unreliable right after kicking. See execute_receiving.
        if self.stable_frame < PassReceive.StabilizationFrames:
            return False
        offset = 0.1
        straight_line = robocup.Point(0, 1)
        pass_segment = self.robot.pos - self.kicked_from
        pass_distance = pass_segment.mag() + 0.5
        pass_dir = pass_segment.normalized()

        left_kick = robocup.Point(-offset, -offset)
        right_kick = robocup.Point(offset, -offset)

        # Create a channel on the left/right of the mouth of the kicker to a bit behind the receiver
        left_recieve = left_kick + straight_line * pass_distance
        right_recieve = right_kick + straight_line * pass_distance

        # Widen the channel to allow for catching the ball.
        left_recieve.rotate(left_kick, PassReceive.MarginAngle)
        right_recieve.rotate(right_kick, -PassReceive.MarginAngle)

        origin = robocup.Point(0, 0)

        passDirRadians = pass_dir.angle()
        left_kick.rotate(origin, passDirRadians - math.pi / 2)
        right_kick.rotate(origin, passDirRadians - math.pi / 2)

        left_recieve.rotate(origin, passDirRadians - math.pi / 2)
        right_recieve.rotate(origin, passDirRadians - math.pi / 2)

        # Add points that create the good_area to a polygon
        good_area = robocup.Polygon()
        good_area.add_vertex(self.kicked_from + left_kick)
        good_area.add_vertex(self.kicked_from + right_kick)

        good_area.add_vertex(self.kicked_from + right_recieve)
        good_area.add_vertex(self.kicked_from + left_recieve)

        main.system_state().draw_raw_polygon(good_area, constants.Colors.Green,
                                             "Good Pass Area")
        return not good_area.contains_point(main.ball().pos)

    def execute_collecting(self):
        # Freeze ball position and velocity once Stabilizationframes is up.
        if self.stable_frame <= PassReceive.StabilizationFrames:
            self.stable_frame = self.stable_frame + 1
            self.reset_correct_location()

        # Alignment will be handled by capture

        ## prefer a robot that's already near the receive position
    def role_requirements(self):
        reqs = super().role_requirements()
        for req in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            if self._target_pos != None:
                req.destination_shape = self._target_pos
            elif self.receive_point != None:
                req.destination_shape = self.receive_point
        return reqs

    def __str__(self):
        desc = super().__str__()
        if self.receive_point != None and self.robot != None:
            desc += "\n    target_pos=" + str(self._target_pos)
            desc += "\n    angle_err=" + str(self._angle_error)
            desc += "\n    x_err=" + str(self._x_error)
            desc += "\n    y_err=" + str(self._y_error)
        return desc
