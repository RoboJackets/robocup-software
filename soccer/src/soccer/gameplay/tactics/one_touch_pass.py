import composite_behavior
import behavior
import skills.move
import tactics.coordinated_pass
import robocup
import constants
import main
import skills.angle_receive
import evaluation.touchpass_positioning
import evaluation.passing
import evaluation.chipping
import enum


## A tactic that causes a robot to pass to another one,
# who scores on the goal as fast as possible.
#
# This class is supplemented by touchpass_positioning and angle_receive
class OneTouchPass(composite_behavior.CompositeBehavior):

    tpass = evaluation.touchpass_positioning
    receivePointChangeThreshold = 0.15  # Percent

    class State(enum.Enum):
        setup = 1
        passing = 2

    def __init__(self, skillkicker=None):
        super().__init__(continuous=False)

        if skillkicker == None:
            skillkicker = skills.pivot_kick.PivotKick()

        self.tpass_iterations = 0
        self.force_reevauation = False

        rp = self.calc_receive_point()

        for state in OneTouchPass.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OneTouchPass.State.passing, lambda: True,
                            'immediately')

        self.add_transition(
            OneTouchPass.State.passing, behavior.Behavior.State.completed,
            lambda: self.pass_bhvr.state == behavior.Behavior.State.completed,
            'Touchpass completed.')

        self.add_transition(OneTouchPass.State.passing,
                            behavior.Behavior.State.failed, lambda: self.
                            pass_bhvr.state == behavior.Behavior.State.failed,
                            'Touchpass failed!')

        self.angle_receive = skills.angle_receive.AngleReceive()

        self.pass_bhvr = tactics.coordinated_pass.CoordinatedPass(
            None,
            self.angle_receive, (skillkicker, lambda x: True),
            receiver_required=False,
            kicker_required=False,
            prekick_timeout=20,
            use_chipper=True)

    def evaluate_chip(self, receive_point):
        bp = main.ball().pos
        ex_robots = self.subbehavior_with_name('pass').get_robots()
        kick_p = evaluation.passing.eval_pass(
            bp, receive_point, excluded_robots=ex_robots)

        if kick_p < .5:
            ex_robots.extend(evaluation.chipping.chippable_robots())
            chip_p = evaluation.passing.eval_pass(
                bp, receive_point, excluded_robots=ex_robots)
            if chip_p > kick_p:
                self.subbehavior_with_name('pass').use_chipper = True

    def calc_receive_point(self):
        ex_robots = list(main.system_state().our_robots)
        ex_robots.extend(evaluation.chipping.chippable_robots())
        receive_pt, _, _ = OneTouchPass.tpass.eval_best_receive_point(
            main.ball().pos, None, ex_robots)
        return receive_pt

    def reset_receive_point(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        ex_robots = pass_bhvr.get_robots()
        ex_robots.extend(evaluation.chipping.chippable_robots())
        receive_pt, target_point, probability = OneTouchPass.tpass.eval_best_receive_point(
            main.ball().pos, None, ex_robots)

        # only change if increase of beyond the threshold.
        if self.force_reevauation or pass_bhvr.receive_point is None or pass_bhvr.target_point is None \
           or probability > OneTouchPass.tpass.eval_single_point(main.ball().pos,
                                                                 pass_bhvr.receive_point, ignore_robots=ex_robots) \
                                                                 + OneTouchPass.receivePointChangeThreshold:
            pass_bhvr.receive_point = receive_pt
            self.pass_bhvr.skillreceiver.target_point = target_point
            self.force_reevauation = False

    def on_enter_passing(self):
        self.angle_receive = skills.angle_receive.AngleReceive()
        self.add_subbehavior(self.pass_bhvr, 'pass', priority=5)

        if self.pass_bhvr.receive_point == None:
            self.reset_receive_point()


    def execute_passing(self):
        if not self.pass_bhvr.state == tactics.coordinated_pass.CoordinatedPass.State.receiving and self.tpass_iterations > 50 or main.ball(
        ).pos.y < self.pass_bhvr.receive_point.y:
            self.force_reevauation = True
            self.reset_receive_point()
            self.tpass_iterations = 0

    def on_exit_passing(self):
        self.remove_subbehavior('pass')
