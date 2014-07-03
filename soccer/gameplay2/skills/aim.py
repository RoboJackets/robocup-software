import single_robot_behavior
import behavior
import enum


# The Aim skill is used when a robot has the ball to aim at a particular target
# It fails if the ball is fumbled
# This behavior is continuous, meaning that once the aim is 'good', it continues running
# rather than entering the 'completed' state.  To indicate that the aim is good, it enters
# the 'aimed' state, which is a substate of running.  It may change back from 'aimed' to
# 'aiming' if it's parameters change or due to external conditions.
class Aim(single_robot_behavior.SingleRobotBehavior):

    class State(enum.Enum):
        aiming = 1
        aimed = 2


    def __init__(self):
        super().__init__(continuous=True)

        self.add_state(Aim.State.aiming, behavior.Behavior.State.running)
        self.add_state(Aim.State.aimed, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            Aim.State.aiming,
            lambda: True,
            'immediately')

        for state in Aim.State:
            self.add_transition(state,
                behavior.Behavior.State.failed,
                lambda: not self.robot.has_ball(),
                'fumble')

        self.add_transition(Aim.State.aiming,
            Aim.State.aimed,
            lambda: self.is_aimed(),
            'error < threshold and not rotating too fast')

        self.add_transition(Aim.State.aimed,
            Aim.State.aiming,
            lambda: not self.is_aimed(),
            'error > threshold or rotating too fast')


        self.use_windowing = True
        self.target = constants.Field.TheirGoalSegment
        self.error_threshold = 0.03
        self.max_steady_ang_vel = 10
        self.dribbler_speed = constants.Robot.Dribbler.MaxPower

        # several different methods rely on these values, which are expensive to calculate
        # we recalculate in execute_running() and cache them in these ivars
        self._aim_target_point = None   # the specific point we're aiming at on the target segment
        self._shot_point = None # the point on the target line we'd hit if we shot now
        self._error = float("inf")


    # If target is a Segment and use_windowing is True, it uses the window evaluator to find the best place to aim at on the Segment
    # Default: True
    @property
    def use_windowing(self):
        return self._use_windowing
    @use_windowing.setter
    def use_windowing(self, value):
        self._use_windowing = value


    # The target Segment or Point that we're aiming at
    # Default: the opponent's goal segment
    @property
    def target(self):
        return self._target
    @target.setter
    def target(self, value):
        self._target = value


    # error threshold is the max distance that we can be off to the side of the target
    # and be considered to have a good aim at it
    # Default: 0.03m
    @property
    def error_threshold(self):
        return self._error_threshold
    @error_threshold.setter
    def error_threshold(self, value):
        self._error_threshold = value
    

    # We don't want to take a shot while we're rotating quickly, we want to be steady
    # This is the max angular velocity (degrees / sec) the robot can have and still be considered to be steady
    # Default: ????
    @property
    def max_steady_ang_vel(self):
        return self._max_steady_ang_vel
    @max_steady_ang_vel.setter
    def max_steady_ang_vel(self, value):
        self._max_steady_ang_vel = value
    

    # Default: full power
    @property
    def dribbler_speed(self):
        return self._dribbler_speed
    @dribbler_speed.setter
    def dribbler_speed(self, value):
        self._dribbler_speed = value
        

    # returns True if we're aimed at our target within our error thresholds and we're not rotating too fast
    def is_aimed(self):
        return self._error < self.error_threshold and self.robot.ang_vel < self.max_steady_ang_vel


    # we're aiming at a particular point on our target segment, what is this point?
    def recalculate(self):
        # find the point we want to aim at
        if isinstance(self.target, robocup.Point):
            self._aim_target_point = self.target
        elif isinstance(self.target, robocup.Segment):
            if use_windowing:
                # FIXME: what if the parent behavior of Aim wants to set other conditions on the window evaluator such as chipping or excluded bots?
                win_eval = evaluation.window_evaluator.WindowEvaluator()
                windows, best = win_eval.eval_pt_to_seg(self.robot.pos, self.target)
                self._aim_target_point = best.center()
            else:
                self._aim_target_point = self.target.center()
        else:
            raise AssertionError("Expected Point or Segment, found: " + str(self.target))

        # find the point we're actually aiming at that's on the line going through aim_target_point
        # and perpendicular to the line from the ball to aim_target_point
        if self._aim_target_point is None:
            self._shot_point = None
        else:
            ball2target = self._aim_target_point - main.ball().pos
            target_line = robocup.Line(self._aim_target_point, self._aim_target_point + ball2target.perp_ccw()) # line perpendicular to aim_line that passes through the target
            aim_line = robocup.Line(self.robot.pos, robocup.Point(math.cos(angle_rad), math.sin(angle_rad)))
            self._shot_point = aim_line.line_intersection(target_line)    # this is the point along target_line that we'll hit if we shoot now

        # error
        if self._aim_target_point != None and self._shot_point != None:
            self._error = (self._aim_target_point - shot_point).mag() if shot_point != None else float("inf") # distance in meters off that we'll be if we shoot right now
        else:
            self._error = float("inf")



    def execute_running(self):
        self.recalculate()

        self.robot.face(self._aim_target_point)
        self.robot.set_dribbler_speed(self.dribbler_speed)

        # draw current shot line
        if self._shot_point != None:
            color = constants.Colors.Green if self.is_aimed() else constants.Colors.Red
            main.system_state().draw_line(robocup.Line(self.robot.pos, self._shot_point), color, "Aim")
            main.system_state().draw_circle(self._shot_point, 0.02, color, "Aim")

        # draw where we're supposed to be aiming
        if self._aim_target_point != None:
            main.system_state().draw_circle(self._aim_target_point, 0.02, constants.Colors.Blue, "Aim")
            if isinstance(self.target, robocup.Segment):
                main.system_state().draw_line(self.target, constants.Colors.Blue, "Aim")
                for i in range(2):
                    main.system_state().draw_line(robocup.Line(self.robot.pos, self.target.pt[i]), constants.Colors.Blue, "Aim")


    def __str__(self):
        desc = super().__str__()
        desc += "\n    err=" + str(self._error) + "m"
        desc += "\n    err thresh=" + str(self.error_threshold) + "m"
        return desc

