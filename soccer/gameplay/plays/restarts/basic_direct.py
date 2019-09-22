import main
import robocup
import behavior
import constants
import enum

import standard_play
import evaluation
import tactics.coordinated_pass
import skills


class BasicDirect(standard_play.StandardPlay):

    #Minimum probability for which a shot should be taken
    MIN_SHOT_PROB = 0.35
    FIELD_LENGTH_MULTIPLIER = 0.75

    class State(enum.Enum):
        kicking = 1
        passing = 2

    def __init__ (self):
        super().__init__(continuous = True)
        for s in BasicDirect.State:
            self.add_state(s, behavior.Behavior.State.running)
        # If chance of scoring is greater than equal to MIN_SHOT_PROB it will kick
        self.add_transition(behavior.Behavior.State.start,
            BasicDirect.State.kicking,
            lambda:evaluation.shooting.eval_shot(main.ball().pos, main.our_robots()) >= BasicDirect.MIN_SHOT_PROB,
            'ShotChanceHigh')        
        #If chance of scoring is below MIN_SHOT_PROB we want to pass using the chipper
        self.add_transition(behavior.Behavior.State.start,BasicDirect.State.passing,
            lambda:evaluation.shooting.eval_shot(main.ball().pos, main.our_robots()) < BasicDirect.MIN_SHOT_PROB,
            'Pass')
        #End play if kicking is done
        self.add_transition(BasicDirect.State.kicking, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('kicking_the_ball').is_done_running(),'Done')
        #End play if chipping is done
        self.add_transition(BasicDirect.State.passing, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('chipping').is_done_running(),'Done')

    def on_enter_kicking(self):
        self.add_subbehavior(skills.pivot_kick.PivotKick(),'kicking_the_ball')

    def on_enter_passing(self):
        kicker = skills.pivot_kick.PivotKick()
        kicker.use_chipper = True
        kicker.kick_power = 50
        #The point the ball will be chipped to
        receive_pt = robocup.Point(0, BasicDirect.FIELD_LENGTH_MULTIPLIER * constants.Field.Length)

        chip_behavior = tactics.coordinated_pass.CoordinatedPass(
            receive_pt, None,
            (kicker, lambda x : True), 
            None, False, False) 
        self.add_subbehavior(chip_behavior,'chipping')

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if behavior.Behavior.State.running or (
            gs.is_ready_state() and gs.is_our_direct_kick()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True