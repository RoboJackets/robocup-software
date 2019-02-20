import robocup
import constants
import standard_play
import enum
import behavior
import main
import evaluation
import tactics.coordinated_pass
import skills


class DirectFreeKick(standard_play.StandardPlay):

    class State(enum.Enum):
        kicking = 1
        passing = 2
    def __init__ (self):
        super().__init__(continuous = True)

        self.add_state(DirectFreeKick.State.kicking,behavior.Behavior.State.running)
        self.add_state(DirectFreeKick.State.passing,behavior.Behavior.State.running)


        self.add_transition(behavior.Behavior.State.start,DirectFreeKick.State.kicking,
            lambda:evaluation.shooting.eval_shot(main.ball().pos, main.our_robots()) >= 0.35,'ShotChanceHigh')        
        self.add_transition(behavior.Behavior.State.start,DirectFreeKick.State.passing,
            lambda:evaluation.shooting.eval_shot(main.ball().pos, main.our_robots()) <0.35,'Pass')

    def on_enter_kicking(self):
        self.add_subbehavior(skills.pivot_kick.PivotKick(),'kicking_the_ball')

    def on_enter_passing(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.use_chipper = True
        kicker.kick_power = 50
        receive_pt = robocup.Point(0, 3 * constants.Field.Length / 4)

        chip_behavior = tactics.coordinated_pass.CoordinatedPass(receive_pt,None,(kicker, lambda x : True), None, False, False) 
        self.add_subbehavior(chip_behavior,'chipping')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state() and gs.is_our_direct_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True