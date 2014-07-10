import single_robot_composite_behavior
import skills._kick
import behavior
import skills.aim
import skills.capture
import robocup
import constants
import main
from enum import Enum


# PivotKick drives up to the ball and captures it, then aims at a specified target and kicks/chips
# Note: PivotKick recalculates aim_target_point from the target at every iteration
class PivotKick(single_robot_composite_behavior.SingleRobotCompositeBehavior, skills._kick._Kick):

    class State(Enum):
        capturing = 1
        aiming = 2
        kicking = 3


    def __init__(self):
        super().__init__()
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
            lambda: self.subbehavior_with_name('aim').state == skills.aim.Aim.State.aimed and self.enable_kick,
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
        self.dribbler_power = constants.Robot.Dribbler.MaxPower
        

    # The speed to drive the dribbler at during aiming
    # If high, adds lift to kick
    # Default: full power
    # FIXME: defaulting to full power probably isn't the best - the C++ version sais full power in the header then actually used 50.  maybe use half speed?
    @property
    def dribbler_power(self):
        return self._dribbler_power
    @dribbler_power.setter
    def dribbler_power(self, value):
        self._dribbler_power = int(value)


    def remove_aim_behavior(self):
        if self.has_subbehavior_with_name('aim'):
            self.remove_subbehavior('aim')


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
        aim.target_point = self.aim_target_point
        aim.dribbler_power = self.dribbler_power


    def on_enter_aiming(self):
        if not self.has_subbehavior_with_name('aim'):
            aim = skills.aim.Aim()
            self.add_subbehavior(aim, 'aim', required=True)
            self.set_aim_params()
    def execute_aiming(self):
        self.recalculate_aim_target_point()
        self.set_aim_params()

        if isinstance(self.target, robocup.Segment):
            for i in range(2):
                main.system_state().draw_line(robocup.Line(main.ball().pos, self.target.get_pt(i)), constants.Colors.Blue, "PivotKick")

    def on_exit_aiming(self):
        # we don't remove the 'aim' subbehavior here because if we're going to the
        # kicking state, we want to keep it around
        pass


    def execute_kicking(self):
        self.recalculate_aim_target_point()
        self.set_aim_params()
        if self.use_chipper and self.robot.has_chipper():
            self.robot.chip(self.chip_power)
        else:
            self.robot.kick(self.kick_power)


    def on_exit_running(self):
        self.remove_aim_behavior()
