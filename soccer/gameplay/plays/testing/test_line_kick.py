import play
import behavior
import skills.pivot_kick
import skills.move
import constants
import robocup


class TestLineKick(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        kick = skills.line_kick.LineKick()
        kick.target = constants.Field.OurGoalSegment
        self.add_subbehavior(kick, 'kick', required=False)

    def execute_running(self):
        kick = self.subbehavior_with_name('kick')
        if kick.is_done_running():
            kick.restart()
