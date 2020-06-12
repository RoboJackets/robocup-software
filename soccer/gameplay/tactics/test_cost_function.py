import robocup
import constants
import math

## A test cost function for passes that should have a minima at (0.915239, 1.38493)
def testCostFunction(coordinates) -> float:
    x, y = coordinates
    cost = 0.4 * ((x - math.sqrt(2))**2 - y) + 0.6 * (-math.sin(x+y))
    return cost