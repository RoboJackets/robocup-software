import skills._kick
import single_robot_composite_behavior
import behavior
import constants
import robocup
import enum
import main
import role_assignment
import evaluation.ball


## lines up with the ball and the target, then drives up and kicks
# this differs from PivotKick which gets the ball first, then aims
# Note: LineKick recalculates the aim_target_point ONLY when the target point/segment changes
#
# See Also: LineKickOld is the old, python-only implementation of line_kick
class LineKick(skills._kick._Kick,
               single_robot_composite_behavior.SingleRobotCompositeBehavior):
    ClosenessThreshold = constants.Robot.Radius * 3 + 0.04

    class State(enum.Enum):
        waiting = 1
        kick = 2

    def __init__(self):
        super().__init__()

        self._got_close = False

        self.add_state(LineKick.State.waiting, behavior.Behavior.State.running)
        self.add_state(LineKick.State.kick, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            LineKick.State.waiting, lambda: True,
                            'immediately')
        self.add_transition(LineKick.State.waiting,
                            LineKick.State.kick, lambda: self.enable_kick,
                            'kicker is enabled')

        self.add_transition(
            LineKick.State.kick,
            behavior.Behavior.State.completed, lambda: self.robot is not None
            and self._got_close and self.robot.just_kicked(), "robot kicked")
        self.shell_id = None

        self.max_speed = None
        self.max_accel = None

    def on_enter_waiting(self):
        self.robot.set_avoid_ball_radius(0.1)
        super().recalculate_aim_target_point()
        self.add_subbehavior(
            skills.move.Move(main.ball().pos - (
                self.aim_target_point - main.ball().pos).normalized() * .5),
            'setupMove')

    def on_exit_waiting(self):
        self.remove_subbehavior('setupMove')
        self.robot.disable_avoid_ball()

    def on_enter_running(self):
        super().recalculate_aim_target_point()

    def execute_kick(self):
        super().execute_running()
        self.robot.disable_avoid_ball()
        self.robot.line_kick(self.aim_target_point)

        if self.max_speed is not None:
            self.robot.set_max_speed(self.max_speed)

        if self.max_accel is not None:
            self.robot.set_max_accel(self.max_accel)

        if main.ball().pos.dist_to(
                self.robot.pos) < LineKick.ClosenessThreshold:
            self._got_close = True

        if self._got_close:
            if self.use_chipper:
                self.robot.chip(self.chip_power)
            else:
                self.robot.kick(self.kick_power)

        # For when ball sense isn't great, kick when vision
        # thinks we have the ball
        if (self.robot is not None and
                evaluation.ball.robot_has_ball(self.robot)):
            self.robot.kick_immediately()

    # TODO Figure out how to renable role requirements for this however it needs to be done
    # def role_requirements(self):
    #     reqs = super().role_requirements()
    #     try to be near the ball
    #     if main.ball().valid:
    #         reqs.destination_shape = main.ball().pos
    #     reqs.require_kicking = True
    #     if self.use_chipper:
    #         reqs.chipper_preference_weight = role_assignment.PreferChipper

    #     if self.shell_id:
    #         reqs.required_shell_id = self.shell_id
    #     return reqs
