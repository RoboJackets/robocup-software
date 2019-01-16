import standard_play
import behavior
import skills.move
import skills.pivot_kick
import constants
import robocup
import math
import main
import tactics.coordinated_pass
import evaluation.passing_positioning
import enum


class IndirectKickoff(standard_play.StandardPlay):

    Running = False
    BumpKickPower = 0.01
    FullKickPower = 1
    MaxShootingAngle = 80
    # Untested as of now
    MaxChipRange = 3
    MinChipRange = 0.3
    
    class State(enum.Enum):
        moving = 1
        passing = 2

    def __init__(self, indirect=None):
        super().__init__(continuous=True)

        # If we are indirect we don't want to shoot directly into the goal
        gs = main.game_state()

        self.add_state(IndirectKickoff.State.moving, behavior.Behavior.State.running)
        self.add_state(IndirectKickoff.State.passing, behavior.Behavior.State.running)

        # self.add_transition(behavior.Behavior.State.start,
        #                     IndirectKickoff.State.moving, lambda: True,
        #                     'immediately')

        # self.add_transition(IndirectKickoff.State.moving, 
        #                     IndirectKickoff.State.passing, 
        #                     lambda: self.moving_done(),
        #                     'Robots are in position to pass')

        # self.add_transition(IndirectKickoff.State.passing,
        #                     behavior.Behavior.State.completed,
        #                     lambda: self.subbehavior_with_name('kicker') == behavior.Behavior.State.completed, 
        #                     'Pass Complete')


        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # FIXME: this could also be a PivotKick
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True

        kicker.kick_power = self.FullKickPower

        # Try passing if we are doing an indirect kick
        receive_pt, receive_value = evaluation.passing_positioning.eval_best_receive_point(main.ball().pos)

        # Check for valid target pass position
        if receive_value != 0:
            pass_behavior = tactics.coordinated_pass.CoordinatedPass(
                receive_pt,
                None,
                (kicker, lambda x: True),
                receiver_required=False,
                kicker_required=False,
                prekick_timeout=9)
            # We don't need to manage this anymore
            self.add_subbehavior(pass_behavior, 'kicker')
        else:
            print('UNFOUND')
            self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

        self.add_transition(
            behavior.Behavior.State.running, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicker').is_done_running() and self.subbehavior_with_name('kicker').state != tactics.coordinated_pass.CoordinatedPass.State.timeout,
            'kicker completes')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if IndirectKickoff.Running or (
            gs.is_ready_state() and gs.is_our_free_kick()) else float("inf")

    # def execute_running(self):
    #     if self.subbehavior_with_name('kicker').state == tactics.coordinated_pass.CoordinatedPass.State.timeout:
    #         self.remove_subbehavior('kicker')
    #         kicker = skills.line_kick.LineKick()
    #         kicker.target = constants.Field.TheirGoalSegment
    #         self.add_subbehavior(kicker, 'kicker', required=False, priority=5)

    #     if self.indirect:
    #         passState = self.subbehavior_with_name('kicker').state
    #         IndirectKickoff.Running = passState == tactics.coordinated_pass.CoordinatedPass.State.receiving or \
    #                               passState == tactics.coordinated_pass.CoordinatedPass.State.kicking

    # def on_enter_running(self):
    #     IndirectKickoff.Running = False

    # def on_exit_running(self):
    #     IndirectKickoff.Running = False

    @classmethod
    def is_restart(cls):
        return True

    def moving_done(self):
        move1_done = self.subbehavior_with_name('move1').state == behavior.Behavior.State.completed
        move1_done = self.subbehavior_with_name('move2').state == behavior.Behavior.State.completed
        return move1_done and move2_done