import single_robot_behavior
import behavior
import main
import robocup
import math
import constants
import enum
import time


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
                lambda: self.fumbled(),
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
        self.error_threshold = 0.04
        self.max_steady_ang_vel = 2
        self.dribbler_speed = int(constants.Robot.Dribbler.MaxPower / 2.0)

        self.last_ball_time = 0

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
        self._dribbler_speed = int(value)
        

    # returns True if we're aimed at our target within our error thresholds and we're not rotating too fast
    def is_aimed(self):
        return self._error < self.error_threshold and self.robot.angle_vel < self.max_steady_ang_vel


    def fumbled(self):
        return not self.robot.has_ball() and time.time() - self.last_ball_time > 0.3


    # we're aiming at a particular point on our target segment, what is this point?
    def recalculate(self):
        # find the point we're actually aiming at that's on the line going through target_point
        # and perpendicular to the line from the ball to target_point
        if self.target_point == None:
            self._shot_point = None
        else:
            ball2target = self.target_point - main.ball().pos
            target_line = robocup.Line(self.target_point, self.target_point + ball2target.perp_ccw()) # line perpendicular to aim_line that passes through the target
            
            # ideally the angle we're aiming at would be the angle of the robot, but the bot doesn't kick straight
            # it tends to kick in the direction of the side of the mouth that the ball is in
            # we draw a line from the center of the bot through the ball and a line along the angle the bot is facing
            # our 'actual' aim line is somewhere in-between the two
            bot_angle_rad = self.robot.angle * constants.DegreesToRadians
            ball_angle_rad = (main.ball().pos - self.robot.pos).angle()
            ball_angle_bias = 0.8   # NOTE: THIS IS TUNABLE
            aim_angle = ball_angle_rad*ball_angle_bias + (1.0 - ball_angle_bias)*bot_angle_rad
            
            # the line we're aiming down
            angle_dir = robocup.Point(math.cos(aim_angle), math.sin(aim_angle))
            aim_line = robocup.Line(self.robot.pos, self.robot.pos + angle_dir)

            # we need to change our face target a bit to account for the difference between bot angle and aim angle
            face_angle_offset = bot_angle_rad - aim_angle
            target_angle_rad = (self.target_point - self.robot.pos).angle()
            face_dir_offset = robocup.Point.direction(target_angle_rad + face_angle_offset)
            self._face_target = self.robot.pos + face_dir_offset
            
            self._shot_point = aim_line.line_intersection(target_line)    # this is the point along target_line that we'll hit if we shoot now

        # error
        if self.target_point != None and self._shot_point != None:
            self._error = (self.target_point - self._shot_point).mag() if self._shot_point != None else float("inf") # distance in meters off that we'll be if we shoot right now
        else:
            self._error = float("inf")


    def execute_running(self):
        if self.robot.has_ball():
            self.last_ball_time = time.time()

        self.recalculate()

        # slowly pivot toward the target
        self.robot.set_max_angle_speed(5)
        self.robot.pivot(self._face_target)
        self.robot.set_dribble_speed(self.dribbler_speed)

        # draw current shot line
        if self._shot_point != None:
            color = constants.Colors.Green if self.is_aimed() else constants.Colors.Red
            main.system_state().draw_line(robocup.Line(self.robot.pos, self._shot_point), color, "Aim")
            main.system_state().draw_circle(self._shot_point, 0.02, color, "Aim")

        # draw where we're supposed to be aiming
        if self.target_point != None:
            main.system_state().draw_circle(self.target_point, 0.02, constants.Colors.Blue, "Aim")


    def __str__(self):
        desc = super().__str__()
        desc += "\n    err=" + str(self._error) + "m"
        desc += "\n    err thresh=" + str(self.error_threshold) + "m"
        return desc


    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.require_kicking = True
        return reqs
