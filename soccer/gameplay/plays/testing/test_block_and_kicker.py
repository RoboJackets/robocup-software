import play
import behavior
import constants
import tactics.coordinated_block
import skills.pivot_kick
import robocup
import main


## Runs our Defense tactic and a pivot kicker than tries to score on our blocking tactic
class TestBlockAndKicker(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = tactics.coordinated_block.CoordinatedBlock()
        self.add_subbehavior(b, name='coordinated block', required=True)

        kick = skills.pivot_kick.PivotKick()

        kick.target = constants.Field.OurGoalSegment
        kick.aim_params['desperate_timeout'] = 3
        self.add_subbehavior(kick, 'kick', required=False)

    def execute_running(self):
        kick = self.subbehavior_with_name('kick')

        sublist = self.subbehavior_with_name(
            'coordinated block').all_subbehaviors()
        roblist = []
        for behavior in sublist:
            roblist.append(behavior.robot)

        kick.shot_obstacle_ignoring_robots = roblist

        if kick.is_done_running():
            kick.restart()

    def on_exit_running(self):
        self.remove_subbehavior('coordinated block')
        self.remove_subbehavior('kick')

    @classmethod
    def handles_goalie(cls):
        return True
