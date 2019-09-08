import play
import behavior
import skills.goalside_mark
import robocup
import main


class TestGoalsideMark(play.Play):
    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def on_enter_running(self):
        b = skills.goalside_mark.Goalside_Mark()
        b.mark_robot = main.their_robots()[0]
        self.add_subbehavior(b, name='goalside mark', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('goalside mark')
