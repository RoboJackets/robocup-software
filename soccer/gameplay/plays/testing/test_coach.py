import play
import behavior
import tactics.positions.coach
import robocup
import main


class TestCoach(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        c = tactics.positions.coach.Coach()
        self.add_subbehavior(c, name='coach', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('coach')

    #we don't want this running in comp
    @classmethod
    def score(cls):
        return 9001

    ## Allow the coach to run during stop.
    @classmethod
    def run_during_stopped(cls):
        return True
