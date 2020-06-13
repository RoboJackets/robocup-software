import robocup
import constants
import math
from typing import Tuple, Callable


## A test cost function for passes that should have a minima at (0.915239, 1.38493)
# Takes in an array of 2 cooridnates
def create_test_cost_function(
        a: float, b: float) -> Callable[[Tuple[float, float]], float]:
    def test_cost_function(coordinates: Tuple[float, float]) -> float:
        x, y = coordinates
        return a * ((x - math.sqrt(2))**2 - y) + b * (-math.sin(x + y))

    return test_cost_function
