import robocup
import constants
import play
import enum
import behavior
import main
import skills.move

# TODO needs to be refactored into a tactic.

## Motivates, encourages, and directs the team.
class Coach(play.Play):

    class State(enum.Enum):
        watching = 0

    def __init__(self):
        super().__init__(continuous=True)
        self.add_state(Coach.State.watching,
                       behavior.Behavior.State.running)
        self.add_transition(behavior.Behavior.State.start, self.State.watching,
                            lambda: True, 'immediately')

    # Demo of moving to a point.
    def on_enter_watching(self):
        move_point = robocup.Point(-constants.Field.Width / 2 - constants.Robot.Radius * 2,
                                   constants.Field.Length / 3)

        move = skills.move.Move(move_point)
        self.add_subbehavior(move, 'watcher')

    def execute_watching(self):
        move = self.subbehavior_with_name('watcher')

        # Face ball
        if (main.ball().valid and move.robot is not None):
            move.robot.face(main.ball().pos)

            # Don't cross to the opponent's side of the field
            max_y = constants.Field.Length / 2 - constants.Robot.Radius * 2
            min_y = constants.Robot.Radius * 2
            move_y_pos = max(min(main.ball().pos.y, max_y), min_y)

            move.pos = robocup.Point(move.pos.x, move_y_pos)

    def on_exit_watching(self):
        self.remove_all_subbehaviors()
