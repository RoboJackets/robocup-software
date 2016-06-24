import play
import behavior
import tactics.positions.goalie
import skills.pivot_kick
import constants
import robocup
import main


## Runs our Defense tactic
class TestGoalie(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        a = tactics.positions.goalie.Goalie()
        self.add_subbehavior(a, name='goalie', required=True)
        a.shell_id = main.root_play().goalie_id

        kick = skills.pivot_kick.PivotKick()
        kick.target = constants.Field.OurGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)

    def execute_running(self):
        kick = self.subbehavior_with_name('kick')
        if kick.is_done_running():
            kick.restart()

    def on_exit_running(self):
        self.remove_subbehavior('goalie')
        self.remove_subbehavior('kick')

    @classmethod
    def handles_goalie(cls):
        return True
