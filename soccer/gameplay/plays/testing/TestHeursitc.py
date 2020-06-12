import main
import robocup
import behavior
import constants
import standard_play
import tactics.coordinated_pass
from tactics.find_pass_point import findPassPoint
from tactics.test_cost_function import testCostFunction

## A test of the pass heuristic system using cost function optimization
# should pass to around (0.915239, 1.38493)
class TestHeuristic(standard_play.StandardPlay):

    def __init__(self) -> None:
        super().__init__(continuous=False)

    def on_enter_start(self) -> None:
        pass_point = findPassPoint(testCostFunction)

        self.add_subbehavior(
            tactics.coordinated_pass.CoordinatedPass(pass_point), 'pass')