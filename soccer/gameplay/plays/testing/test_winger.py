import play
import behavior
import constants
import robocup
import tactics.positions.wing_defender as wd
import main
import skills

class TestWinger(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running, lambda: True,
            'now')

        defender = wd.WingDefender(mark_robot = main.system_state().their_robots[0])
        self.add_subbehavior(defender, "wing_test")
