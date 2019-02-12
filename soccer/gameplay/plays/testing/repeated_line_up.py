import play
import tactics.line_up
import behavior_sequence
import tools.sleep
import robocup
import constants
import time
import enum


## Robots repeatedly line up on opposite sides of the field
class RepeatedLineUp(play.Play):
    class State(enum.Enum):
        left = 0
        right = 1
        pause = 2

    PAUSE = 2.0
    BUFFER = .7

    def __init__(self):
        super().__init__(continuous=True)

        self.side_start = time.time()

        behaviors = [
            tactics.line_up.LineUp(self.generate_line(-RepeatedLineUp.BUFFER)),
            tools.sleep.SleepBehavior(RepeatedLineUp.PAUSE),
            tactics.line_up.LineUp(self.generate_line(RepeatedLineUp.BUFFER)),
            tools.sleep.SleepBehavior(RepeatedLineUp.PAUSE),
        ]
        b = behavior_sequence.BehaviorSequence(
            continuous=True, repeat=True, behaviors=behaviors)
        self.add_subbehavior(b, 'line up behavior')

    # x_multiplier is a 1 or -1 to indicate which side of the field to be on
    # 1 is right, -1 is left
    def generate_line(self, x_multiplier):
        x = (constants.Field.Width / 2 - constants.Robot.Radius * 2
             ) * x_multiplier
        y_start = 1.2
        line = robocup.Segment(
            robocup.Point(x, constants.Robot.Radius + y_start),
            robocup.Point(x,
                          (constants.Robot.Radius * 2.3 + 0.1) * 6 + y_start))
        return line
