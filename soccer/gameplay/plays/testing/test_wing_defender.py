import play
import behavior
import constants
import robocup
import tactics.positions.wing_defender
import main

class TestWingDefender(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        tact = tactics.positions.wing_defender.WingDefender(main.ball().pos)
        self.add_subbehavior(tact, 'tact', required=False)

    def execute_running(self):
        tact = self.subbehavior_with_name('tact')

    def on_exit_running(self):
        self.remove_subbehavior('tact')