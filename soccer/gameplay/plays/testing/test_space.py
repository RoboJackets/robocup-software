import main
import play
import behavior
import evaluation.field
import constants
import robocup


class TestSpace(play.Play):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        print(evaluation.field.space_coeff_at_pos(robocup.Point(0, constants.Field.Length/2), main.our_robots()))
