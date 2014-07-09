import single_robot_behavior
import behavior
import constants
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


        self.target_point = constants.Field.TheirGoalSegment.center()
        self.error_threshold = 0.03
        self.max_steady_ang_vel = 10
        self.dribbler_speed = constants.Robot.Dribbler.MaxPower

        # several different methods rely on these values, which are expensive to calculate
        # we recalculate in execute_running() and cache them in these ivars
        self._shot_point = None # the point on the target line we'd hit if we shot now
        self._error = float("inf")


    # The target Point that we're aiming at
    # Default: the center of the opponent's goal segment
    @property
    def target_point(self):
        return self._target_point
    @target_point.setter
    def target_point(self, value):
        self._target_point = value


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
        # find the point we're actually aiming at that's on the line going through target_point
        # and perpendicular to the line from the ball to target_point
        if self.target_point is None:
            self._shot_point = None
        else:
            ball2target = self.target_point - main.ball().pos
            target_line = robocup.Line(self.target_point, self.target_point + ball2target.perp_ccw()) # line perpendicular to aim_line that passes through the target
            aim_line = robocup.Line(self.robot.pos, robocup.Point(math.cos(angle_rad), math.sin(angle_rad)))
            self._shot_point = aim_line.line_intersection(target_line)    # this is the point along target_line that we'll hit if we shoot now

        # error
        if self.target_point != None and self._shot_point != None:
            self._error = (self.target_point - shot_point).mag() if shot_point != None else float("inf") # distance in meters off that we'll be if we shoot right now
        else:
            self._error = float("inf")


    def execute_running(self):
        self.recalculate()

        self.robot.face(self.target_point)
        self.robot.set_dribbler_speed(self.dribbler_speed)

        # draw current shot line
        if self._shot_point != None:
            color = constants.Colors.Green if self.is_aimed() else constants.Colors.Red
            main.system_state().draw_line(robocup.Line(self.robot.pos, self._shot_point), color, "Aim")
            main.system_state().draw_circle(self._shot_point, 0.02, color, "Aim")

        # draw where we're supposed to be aiming
        if self.target_point != None:
            main.system_state().draw_circle(self.target_point, 0.02, constants.Colors.Blue, "Aim")
            if isinstance(self.target, robocup.Segment):
                main.system_state().draw_line(self.target, constants.Colors.Blue, "Aim")
                for i in range(2):
                    main.system_state().draw_line(robocup.Line(self.robot.pos, self.target.get_pt()[i]), constants.Colors.Blue, "Aim")


    def __str__(self):
        desc = super().__str__()
        desc += "\n    err=" + str(self._error) + "m"
        desc += "\n    err thresh=" + str(self.error_threshold) + "m"
        return desc

