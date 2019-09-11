import main
import play
import behavior
import constants
import robocup

import tactics.positions.wing_defender as wing_defender
import evaluation.opponent as opponent_eval


## Tests wing defender on the closest opponent robot to our goal
class TestWingDefender(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # Defend against robot closest to the goal
        bot = opponent_eval.get_closest_opponent(robocup.Point(0, 0))
        tact = wing_defender.WingDefender(mark_robot=bot)
        self.add_subbehavior(tact, 'tact', required=False)

    def execute_running(self):
        tact = self.subbehavior_with_name('tact')

    def on_exit_running(self):
        self.remove_subbehavior('tact')
