import robocup
import positions.position
import behavior
import skills.move
import enum

# Simple goalie who chills in goal
class SimpleGoalie(positions.position.Position):
    def __init__(self, position_class: enum.Enum, name: str):
        super().__init__(position_class, name)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running,
                            lambda: True, 'immediately')

        self.move = skills.move.Move()
        self.add_subbehavior(self.move, "move", False, 0)

    def execute_running(self):
        self.move.pos = robocup.Point(0,0)