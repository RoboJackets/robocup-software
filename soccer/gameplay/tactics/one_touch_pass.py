import composite_behavior
import behavior
import skills.move
import tactics.coordinated_pass
import robocup
import constants
import main
import skills.angle_receive
import evaluation.touchpass_positioning
import enum


## A tactic that causes a robot to pass to another one,
# who scores on the goal as fast as possible.
#
# This class is supplemented by touchpass_positioning and angle_receive
class OneTouchPass(composite_behavior.CompositeBehavior):

    tpass = evaluation.touchpass_positioning
    receivePointChangeThreshold = 0.15  # 15%

    class State(enum.Enum):
        passing = 1

    def __init__(self):
        super().__init__(continuous=False)

        self.tpass_iterations = 0
        self.force_reevauation = False

        for state in OneTouchPass.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            OneTouchPass.State.passing, lambda: True,
                            'immediately')

        self.add_transition(
            OneTouchPass.State.passing, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed,
            'Touchpass completed.')

        self.add_transition(
            OneTouchPass.State.passing, behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.failed,
            'Touchpass failed!')

    def reset_receive_point(self):
        angle_receive = skills.angle_receive.AngleReceive()

        pass_bhvr = self.subbehavior_with_name('pass')
        receive_pt, target_point, probability = OneTouchPass.tpass.eval_best_receive_point(
            main.ball().pos, None, pass_bhvr.get_robots())
        # only change if increase of beyond the threshold.
        if self.force_reevauation == True or pass_bhvr.receive_point == None or pass_bhvr.target_point == None or probability > OneTouchPass.tpass.eval_single_point(
                main.ball().pos, pass_bhvr.receive_point, pass_bhvr.get_robots(
                )) + OneTouchPass.receivePointChangeThreshold:
            pass_bhvr.receive_point = receive_pt
            angle_receive.target_point = target_point
            self.force_reevauation = False

        pass_bhvr.skillreceiver = angle_receive

    def on_enter_passing(self):
        pass_bhvr = tactics.coordinated_pass.CoordinatedPass()
        self.add_subbehavior(pass_bhvr, 'pass')
        if pass_bhvr.receive_point == None:
            self.reset_receive_point()

    def execute_passing(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        self.tpass_iterations = self.tpass_iterations + 1
        if not pass_bhvr.state == tactics.coordinated_pass.CoordinatedPass.State.receiving and self.tpass_iterations > 50 or main.ball(
        ).pos.y < pass_bhvr.receive_point.y:
            self.force_reevauation = True
            self.reset_receive_point()
            self.tpass_iterations = 0

    def on_exit_passing(self):
        self.remove_subbehavior('pass')
