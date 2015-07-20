import single_robot_composite_behavior
import skills._kick
import behavior
import skills.aim
import skills.capture
import role_assignment
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
        aimed = 3
        kicking = 4


    def __init__(self):
        super().__init__()

        for state in PivotKick.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            PivotKick.State.capturing,
            lambda: True,
            'immediately')
        self.add_transition(PivotKick.State.capturing,
            PivotKick.State.aiming,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'has ball')

        self.add_transition(PivotKick.State.aiming,
            PivotKick.State.aimed,
            lambda: self.subbehavior_with_name('aim').state == skills.aim.Aim.State.aimed,
            'aim error < threshold')

        self.add_transition(PivotKick.State.aimed,
            PivotKick.State.aiming,
            lambda: self.subbehavior_with_name('aim').state == skills.aim.Aim.State.aiming and not self.enable_kick,
            'aim error > threshold')
        
        self.add_transition(PivotKick.State.aimed,
            PivotKick.State.kicking,
            lambda: self.enable_kick,
            'kick enabled')

        self.add_transition(PivotKick.State.kicking,
            behavior.Behavior.State.completed,
            lambda: self.robot.just_kicked(),
            'kick complete')

        self.add_transition(PivotKick.State.aiming,
            PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed,
            'fumble')
        self.add_transition(PivotKick.State.aimed,
            PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed,
            'fumble')
        self.add_transition(PivotKick.State.kicking,
            PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed and not self.robot.just_kicked(),
            'fumble')


        # default parameters
        self.dribbler_power = constants.Robot.Dribbler.MaxPower
        self.aim_params = {'desperate_timeout': float("inf")}
        

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


    # if you want to set custom error thresholds, etc to be used during aiming,
    # set this to a dictionary with the appropriate keys and values
    # default: {'desperate_timeout': float("inf")}
    @property
    def aim_params(self):
        return self._aim_params
    @aim_params.setter
    def aim_params(self, value):
        self._aim_params = value
    

    # The point near the target point that we're currently aimed at, whether we want to be or not
    # If we kicked right now, the ball would pass through this point
    def current_shot_point(self):
        if self.has_subbehavior_with_name('aim'):
            return self.subbehavior_with_name('aim').current_shot_point()
        else:
            return None


    def is_steady(self):
        if self.has_subbehavior_with_name('aim'):
            return self.subbehavior_with_name('aim').is_steady()
        else:
            return None
    

    def remove_aim_behavior(self):
        if self.has_subbehavior_with_name('aim'):
            self.remove_subbehavior('aim')


    def on_enter_capturing(self):
        self.remove_aim_behavior()
        self.robot.unkick()
        capture = skills.capture.Capture()
        capture.dribbler_power = self.dribbler_power
        self.add_subbehavior(capture, 'capture', required=True)
        # FIXME: tell capture to approach from a certain direction so we're already lined up?
    def on_exit_capturing(self):
        self.remove_subbehavior('capture')


    def set_aim_params(self):
        aim = self.subbehavior_with_name('aim')
        aim.target_point = self.aim_target_point
        aim.dribbler_power = self.dribbler_power
        for key, value in self.aim_params.items():
            setattr(aim, key, value)


    def execute_running(self):
        self.recalculate_aim_target_point()
        super().execute_running()


    def on_enter_aiming(self):
        if not self.has_subbehavior_with_name('aim'):
            aim = skills.aim.Aim()
            self.add_subbehavior(aim, 'aim', required=True)
            self.set_aim_params()

    def execute_aiming(self):
        self.set_aim_params()

        if isinstance(self.target, robocup.Segment):
            for i in range(2):
                main.system_state().draw_line(robocup.Line(main.ball().pos, self.target.get_pt(i)), constants.Colors.Blue, "PivotKick")

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



    def role_requirements(self):
        reqs = super().role_requirements()

        for r in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            # try to be near the ball
            if main.ball().valid:
                r.destination_shape = main.ball().pos
            if self.use_chipper:
                r.chipper_preference_weight = role_assignment.PreferChipper
            r.require_kicking = True

        return reqs