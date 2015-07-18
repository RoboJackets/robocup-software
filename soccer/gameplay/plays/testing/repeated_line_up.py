import play
import behavior
import tactics.line_up
import robocup
import constants
import enum
import time


## Robots repeatedly line up on opposite sides of the field
class RepeatedLineUp(play.Play):

    Pause = 2.0


    class State(enum.Enum):
        left = 0
        right = 1
        pause = 2


    def __init__(self):
        super().__init__(continuous=True)

        self.side_start = time.time()

        for state in RepeatedLineUp.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            RepeatedLineUp.State.right,
            lambda: True,
            'immediately')

        self.add_transition(RepeatedLineUp.State.left,
            RepeatedLineUp.State.pause,
            lambda: self.subbehavior_with_name('LineUp').state == behavior.Behavior.State.completed and time.time() - self.side_start > 1,
            'made it to left')

        self.add_transition(RepeatedLineUp.State.right,
            RepeatedLineUp.State.pause,
            lambda: self.subbehavior_with_name('LineUp').state == behavior.Behavior.State.completed and time.time() - self.side_start > 1,
            'made it to right')

        self.add_transition(RepeatedLineUp.State.pause,
            RepeatedLineUp.State.right,
            lambda: (time.time() - self.pause_start_time) > RepeatedLineUp.Pause and self.prev_side == RepeatedLineUp.State.left,
            'pause over')
        self.add_transition(RepeatedLineUp.State.pause,
            RepeatedLineUp.State.left,
            lambda: (time.time() - self.pause_start_time) > RepeatedLineUp.Pause and self.prev_side == RepeatedLineUp.State.right,
            'pause over')


    def on_enter_left(self):
        self.side_start = time.time()
        self.add_subbehavior(tactics.line_up.LineUp(self.generate_line(-1)), 'LineUp')
    def on_exit_left(self):
        self.remove_subbehavior('LineUp')
        self.prev_side = RepeatedLineUp.State.left

    def on_enter_right(self):
        self.side_start = time.time()
        self.add_subbehavior(tactics.line_up.LineUp(self.generate_line(1)), 'LineUp')
    def on_exit_right(self):
        self.remove_subbehavior('LineUp')
        self.prev_side = RepeatedLineUp.State.right

    def on_enter_pause(self):
        self.pause_start_time = time.time()


    # x_multiplier is a 1 or -1 to indicate which side of the field to be on
    # 1 is right, -1 is left
    def generate_line(self, x_multiplier):
        x = (constants.Field.Width/2 - constants.Robot.Radius*2) * x_multiplier
        y_start = 1.0
        line = robocup.Segment(
                robocup.Point(x, constants.Robot.Radius + y_start),
                robocup.Point(x, (constants.Robot.Radius * 2.3 + 0.1)*6 + y_start))
        return line
