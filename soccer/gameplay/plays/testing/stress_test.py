import play
import skills
import robocup
import behavior
import constants
import main


# Makes a robot continually run laps
# note: only really works with one robot on the field at a time
class StressTest(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.points = [robocup.Point(constants.Field.Width * 1.0 / 3.0,
                                     constants.Field.Length / 6.0),
                       robocup.Point(-constants.Field.Width * 1.0 / 3.0,
                                     constants.Field.Length * 2.0 / 6.0),
                       robocup.Point(constants.Field.Width * 1.0 / 3.0,
                                     constants.Field.Length * 2.0 / 6.0),
                       robocup.Point(-constants.Field.Width * 1.0 / 3.0,
                                     constants.Field.Length / 6.0)]
        self.index = 0

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def on_enter_running(self):
        m = skills.move.Move()
        self.add_subbehavior(m, 'move', required=True)

    def on_exit_running(self):
        self.remove_subbehavior('move')

    def execute_running(self):
        # draw laps
        # indices = list(range(len(self.points))) + [0]
        # for i in range(len(indices)):
        main.system_state().draw_line(
            robocup.Line(self.points[0], self.points[1]), (255, 0, 0),
            "StressTest")

        m = self.subbehavior_with_name('move')
        if m.state == behavior.Behavior.State.completed:
            # increment index
            self.index = (self.index + 1) % len(self.points)
        m.pos = self.points[self.index]
