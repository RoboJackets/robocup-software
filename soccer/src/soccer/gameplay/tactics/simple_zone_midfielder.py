import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import skills.move


# 2 offenders stay at X% of the Y coordinate of the ball
class SimpleZoneMidfielder(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        hold = 1

    def __init__(self):
        super().__init__(continuous=True)

        self.y_hold_percent = .8
        self.x_positions = [-constants.Field.Width / 3, \
                            constants.Field.Width / 3]
        self.names = ['left_mid', 'right_mid']

        self.moves = [None, None]

        for s in SimpleZoneMidfielder.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            SimpleZoneMidfielder.State.hold, lambda: True,
                            'immediately')

    # Continue updating the mark positions
    def execute_hold(self):
        target_y = self.y_hold_percent * main.ball().pos.y

        for i in range(2):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(robocup.Point(
                    self.x_positions[i], target_y))
                self.add_subbehavior(self.moves[i],
                                     self.names[i],
                                     required=False,
                                     priority=1)
            else:
                self.moves[i].pos = robocup.Point(self.x_positions[i],
                                                  target_y)

    def on_exit_hold(self):
        self.remove_all_subbehaviors()
