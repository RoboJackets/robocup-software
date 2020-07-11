import play
import behavior
import constants
import robocup
import tactics.wall
import main

class TestWall(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        tact = tactics.wall.Wall()
        self.add_subbehavior(tact, 'tact', required=False)

    def execute_running(self):
        tact = self.subbehavior_with_name('tact')
        tact.mark_point = main.ball().pos

    def on_exit_running(self):
        self.remove_subbehavior('tact')