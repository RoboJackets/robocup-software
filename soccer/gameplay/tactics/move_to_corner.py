import main
import robocup
import behavior
import constants
import enum
import math

import composite_behavior
import skills.move


# 2 offenders stay at X% of the Y coordinate of the ball
class MoveToCorner(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        hold = 1

    def __init__(self):
        super().__init__(continuous=True)

        self.corners = [robocup.Point(-1.5, 3.0),robocup.Point(-1.5, 6.0), robocup.Point(1.5, 6.0), robocup.Point(1.5, 3.0)]

        self.names = ['clockwise', 'counterclockwise']

        self.pass_corner = 0

        self.moves = [None, None]

        self.direction = [1, -1]

        for s in MoveToCorner.State:
            self.add_state(s, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            MoveToCorner.State.hold, lambda: True,
                            'immediately')

    # Continue updating the mark positions
    def execute_hold(self):

        for i in range(2):
            if (self.moves[i] is None):
                self.moves[i] = skills.move.Move(
                    self.corners[(i + self.direction[i]) % 4])
                self.add_subbehavior(self.moves[i],
                                     self.names[i],
                                     required=False,
                                     priority=1)
            else:
                self.moves[i].pos = self.corners[(i + self.direction[i]) % 4]

    def on_exit_hold(self):
        self.remove_all_subbehaviors()

    @property
    def pass_corner(self):
        return self._pass_corner

    @pass_corner.setter
    def pass_corner(self, value):
        self._pass_corner = value