import single_robot_composite_behavior
import skills._kick
import behavior
import skills.aim
import skills.capture
import robocup
import constants
from enum import Enum


# PivotKick drives up to the ball and captures it, then aims at a specified target and kicks/chips
class PivotKick(single_robot_composite_behavior.SingleRobotCompositeBehavior, skills._kick._Kick):

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
            'immediately')
        self.add_transition(PivotKick.State.capturing,
            PivotKick.State.aiming,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'has ball')
        self.add_transition(PivotKick.State.aiming,
            PivotKick.State.kicking,
            lambda: self.subbehavior_with_name('aim').state == aim.Aim.State.aimed and self.enable_kick,
            'aim error < threshold and kick enabled')
        self.add_transition(PivotKick.State.kicking,
            behavior.Behavior.State.completed,
            lambda: self.robot.just_kicked(),
            'kick complete')

        self.add_transition(PivotKick.State.aiming,
            PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed,
            'fumble')
        self.add_transition(PivotKick.State.kicking,
            PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed and not self.robot.just_kicked(),
            'fumble')


        # default parameters
        self.target_segment = constants.Field.TheirGoalSegment
        self.use_windowing = True
        self.dribbler_power = constants.Robot.Dribbler.MaxPower
        


    # defaults to opponent's goal
    # NOTE: PivotKick doesn't do windowing or smart shooting at this target
    #       The parent behavior should set this target smartly (probably based on results from WindowEvaluator)
    # if use_windowing is True, chooses the best place to aim along the segment, otherwise aims at the center
    @property
    def target_segment(self):
        return self._target_segment
    @target_segment.setter
    def target_segment(self, value):
        self._target_segment = value


    # if True, uses the window evaluator to choose the best place to aim at target_segment
    # Default: True
    # FIXME: right now when using windowing, we don't take into account chipping over bots
    @property
    def use_windowing(self):
        return self._use_windowing
    @use_windowing.setter
    def use_windowing(self, value):
        self._use_windowing = value


    # The speed to drive the dribbler at during aiming
    # If high, adds lift to kick
    # Default: full power
    # FIXME: defaulting to full power probably isn't the best - the C++ version sais full power in the header then actually used 50.  maybe use half speed?
    @property
    def dribbler_power(self):
        return self._dribbler_power
    @dribbler_power.setter
    def dribbler_power(self, value):
        self._dribbler_power = value


    def remove_aim_behavior(self):
        if self.has_subbehavior_with_name('aim'):
            self.remove_subbehavior_with_name('aim')


    def on_enter_capturing(self):
        self.remove_aim_behavior()
        self.robot.unkick()
        capture = skills.capture.Capture()
        self.add_subbehavior(capture, 'capture', required=True)
        # FIXME: tell capture to approach from a certain direction so we're already lined up?
    def on_exit_capturing(self):
        self.remove_subbehavior('capture')


    def set_aim_params(self):
        aim = self.subbehavior_with_name('aim')
        aim.use_windowing = self.use_windowing
        aim.target = self.target_segment
        aim.dribbler_power = self.dribbler_power


    def on_enter_aiming(self):
        if not self.has_subbehavior_with_name('aim'):
            aim = skills.aim.Aim()
            self.add_subbehavior(aim, 'aim', required=True)
            self.set_aim_params()
    def execute_aiming(self):
        self.set_aim_params()
    def on_exit_aiming(self):
        # we don't remove the 'aim' subbehavior here because if we're going to the
        # kicking state, we want to keep it around
        pass


    def execute_kicking(self):
        self.set_aim_params()
        if self.use_chipper and self.robot.has_chipper():
            self.robot.chip(self.chip_power)
        else:
            self.robot.kick(self.kick_power)


    def on_exit_running(self):
        self.remove_aim_behavior()

