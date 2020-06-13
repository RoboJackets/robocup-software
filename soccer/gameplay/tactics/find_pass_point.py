import robocup
import constants
from scipy.optimize import minimize
from typing import Callable, Tuple, Optional, Any, Sequence


## Function that optimizes cost function for passing
# uses scipy minimize using the BFGS method
def find_pass_point(heuristic: Callable[[Tuple[float, float]], float],
                    initial_guess: Optional[Tuple[float, float]] = (0, 0),
                    max_iter: Optional[int] = None) -> robocup.Point:
    width = constants.Field.Width
    length = constants.Field.Length
    if (max_iter is not None):
        options = {'maxiter': max_iter, 'disp': False}
    else:
        options = None
    result = minimize(heuristic,
                      initial_guess,
                      bounds=((-width / 2.00, width / 2.00), (0.00, length)),
                      tol=1e-3,
                      options=options)
    # if (not result.success):
    #     raise OptimizationError()
    point = robocup.Point(result.x[0], result.x[1])
    return point


## Exception for when optimization fails
# without throwing an exception the bot will just pass to 0,0
# class OptimizationError(Exception):
#     def __init__(
#         self,
#         message='Error during optimization of cost function, please check expression being optimized for mathematical errors'
#     ) -> None:
#         self.message = message
#         super().__init__(self.message)
