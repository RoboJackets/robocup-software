import play
import behavior
import tactics.positions.celebration
import robocup
import main


class TestCelebration(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        c = tactics.positions.celebration.Celebration()
        self.add_subbehavior(c, name='celebrate', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('celebrate')

    #we don't want this running in comp
    @classmethod
    def score(cls):
        return 9001

    ## Allow the coach to run during stop.
    @classmethod
    def run_during_stopped(cls):
        return True
