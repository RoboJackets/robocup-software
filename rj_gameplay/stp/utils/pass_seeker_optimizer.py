from scipy.optimize import minimize
import numpy as np
from typing import Callable, Tuple, Optional, Any, Sequence
import stp.rc as rc


## Function that optimizes cost function for pass seeking
# uses scipy minimize using the BFGS method
def find_seek_point(heuristic: Callable[[Tuple[float, float]], float],
                    world_state: rc.WorldState,
                    initial_guess: Optional[Tuple[float, float]] = (0, 0),
                    max_iter: Optional[int] = None) -> np.ndarray:
    width = world_state.field.width_m
    length = world_state.field.length_m
    if (max_iter is not None):
        options = {'maxiter': max_iter, 'disp': False}
    else:
        options = None
    result = minimize(heuristic,
                      initial_guess, (world_state),
                      bounds=((-width / 2.00, width / 2.00), (0.00, length)),
                      tol=1e-3,
                      options=options)
    point = np.array([result.x[0], result.x[1]])
    return point
