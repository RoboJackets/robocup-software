import play
import behavior
import skills.move
import robocup
import main
import constants


class TestPriority(play.Play):
    MAX_COUNTER_VAL = 500

    def __init__(self):
        super().__init__(continuous=True)

        self.cur_priority = 80
        self.counter = 0

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    # Moves all the robots from one side to the other depending on which priority is higher
    def on_enter_running(self):
        inc = robocup.Point(0, .5)
        left = robocup.Point(-1, 1)
        right = robocup.Point(1, 1)

        for i in range(6):
            self.add_subbehavior(
                skills.move.Move(left + inc * i),
                name='move' + str(i),
                required=False,
                priority=90)

        for i in range(6):
            self.add_subbehavior(
                skills.move.Move(right + inc * i),
                name='movel' + str(i),
                required=False,
                priority=lambda: self.cur_priority)

    def execute_running(self):
        self.counter += 1

        if self.counter > TestPriority.MAX_COUNTER_VAL:
            if self.cur_priority < 90:
                self.cur_priority = 100
            else:
                self.cur_priority = 80
            self.counter = 0

    def on_exit_running(self):
        self.remove_all_subbehaviors()
