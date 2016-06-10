import skills._kick
import behavior
import constants
import robocup
import enum
import main
import role_assignment


## lines up with the ball and the target, then drives up and kicks
# this differs from PivotKick which gets the ball first, then aims
# Note: LineKick recalculates the aim_target_point ONLY when the target point/segment changes
class LineKick(skills._kick._Kick):
    ClosenessThreshold = constants.Robot.Radius + 0.04

    class State(enum.Enum):
        setup = 1
        charge = 2

    def __init__(self):
        super().__init__()

        self._got_close = False

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True, 'immediately')

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.robot is not None and self._got_close and self.robot.just_kicked(),
            "robot kicked")


    def on_enter_running(self):
        super().recalculate_aim_target_point()

    def execute_running(self):
        super().execute_running()
        self.robot.line_kick(self.aim_target_point)

        if main.ball().pos.dist_to(
                self.robot.pos) < LineKick.ClosenessThreshold:
            self._got_close = True

        if self._got_close:
            if self.use_chipper:
                self.robot.chip(self.chip_power)
            else:
                self.robot.kick(self.kick_power)

    def role_requirements(self):
        reqs = super().role_requirements()
        # try to be near the ball
        if main.ball().valid:
            reqs.destination_shape = main.ball().pos
        reqs.require_kicking = True
        if self.use_chipper:
            reqs.chipper_preference_weight = role_assignment.PreferChipper
        return reqs
