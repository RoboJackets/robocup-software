import play
import behavior
import skills.pivot_kick
import skills.move
import enum
import main
import constants
import robocup


# this test repeatedly runs the PivotKick behavior aimed at our goal
class TestPivotKick(play.Play):

    class State(enum.Enum):
        waiting = 1
        kicking = 2

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.OurGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)


    def execute_running(self):
        kick = self.subbehavior_with_name('kick')
        if kick.is_done_running():
            kick.restart()
