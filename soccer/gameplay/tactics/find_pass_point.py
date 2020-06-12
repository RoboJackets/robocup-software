import robocup
import constants
from scipy.optimize import minimize

## Function that optimizes cost function for passing
# uses scipy minimize using the BFGS method
def findPassPoint(heuristic) -> robocup.Point:
    width = constants.Field.Width
    length = constants.Field.Length
    initial_guess = [0.00, 0.00]
    result = minimize(heuristic, initial_guess, bounds=((-width/2.00, width/2.00), (0.00, length)), tol=1e-3)
    if (not result.success):
        raise OptimizationError()
    point = robocup.Point(result.x[0], result.x[1])
    print(point)
    return point

## Exception for when optimization fails
# without throwing an exception the bot will just pass to 0,0
class OptimizationError(Exception):

    def __init__(self, message='Error during optimization of cost function, please check expression being optimized for mathematical errors'):
        self.message = message
        super().__init__(self.message)
