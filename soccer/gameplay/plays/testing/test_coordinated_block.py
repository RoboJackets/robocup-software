import play
import behavior
import tactics.coordinated_block
import robocup
import main


## Runs our Coordinated Block tactic
class TestCoordinatedBlock(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = tactics.coordinated_block.CoordinatedBlock()
        self.add_subbehavior(b, name='coordinated block', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('coordinated block')

    @classmethod
    def handles_goalie(cls):
        return True
