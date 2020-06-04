import main
import robocup
import behavior
import constants

import standard_play
import evaluation
import situational_play_selection
import tactics.coordinated_pass
from tactics.pass_to_point_heuristic import pass_to_point_heuristic


##
# A test play that just uses the pass to point heuristic that takes in coodrinates and passes there
#
class TestHeuristic(standard_play.StandardPlay):
    def __init__(self) -> None:
        super().__init__(continuous=False)

    def on_enter_start(self) -> None:
        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(
                pass_to_point_heuristic(constants.Field.Width / 2,
                                        constants.Field.Length / 2)), 'pass')
