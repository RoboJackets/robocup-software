import skill
import behavior
import constants
import robocup
from enum import Enum


class PivotKick(skill.Skill):

    # tunable parameters
    ################################################################################
    AimSpeed = 1 # FIXME: bogus value
    AimThreshold = 1 # FIXME: bogus value



    class State(Enum):
        capturing = 1
        aiming = 2
        kicking = 3


    def __init__(self):
        super().__init__(continuous=False)
        self.add_state(PivotKick.State.capturing, behavior.Behavior.State.running)
        self.add_state(PivotKick.State.aiming, behavior.Behavior.State.running)
        self.add_state(PivotKick.State.kicking, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            PivotKick.State.capturing,
            lambda: True,
            'immediate')
        self.add_transition(PivotKick.State.capturing,
            PivotKick.State.aiming,
            lambda: True,
            'has ball')
        self.add_transition(PivotKick.State.aiming,
            PivotKick.State.kicking,
            lambda: True,
            'aim error < threshold')
        self.add_transition(PivotKick.State.kicking,
            behavior.Behavior.State.completed,
            lambda: True,
            'kick complete')

        self.add_transition(PivotKick.State.aiming,
            PivotKick.State.capturing,
            lambda: False,
            'fumble')
        self.add_transition(PivotKick.State.kicking,
            PivotKick.State.capturing,
            lambda: False,
            'fumble')


        # default parameters
        self.target_segment = robocup.Segment(robocup.Point(constants.Field.Length, -constants.Field.GoalWidth / 2.0),
                                                robocup.Point(constants.Field.Length, constants.Field.GoalWidth / 2.0))
        self.user_chipper = False

        self.kick_power = 255
        self.dribbler_speed = 50


    def execute_capturing(self):
        pass

    def execute_aiming(self):
        pass

    def execute_kicking(self):
        pass


    # defaults to opponent's goal
    # FIXME: does it aim at the center? anywhere?
    @property
    def target_segment(self):
        return self._target_segment
    @target_segment.setter
    def target_segment(self, value):
        self._target_segment = value


    # The speed to drive the dribbler at during aiming
    # If high, adds lift to kick
    # Default: full speed
    @property
    def dribbler_speed(self):
        return self._dribbler_speed
    @dribbler_speed.setter
    def dribbler_speed(self, value):
        self._dribbler_speed = value


    # If false, uses straight kicker, if true, uses chipper
    # Default: false
    @property
    def use_chipper(self):
        return self._use_chipper
    @use_chipper.setter
    def use_chipper(self, value):
        self._use_chipper = value


    # Allows for different kicker settings, such as for
    # passing with lower power.
    # Default: 255 - full power
    @property
    def kick_power(self):
        return self._kick_power
    @kick_power.setter
    def kick_power(self, value):
        self._kick_power = value



