"""This module contains a variety of functions that return constraint functions for
convenience."""

from typing import Optional

import stp.rc as rc
import stp.role as role


def has_ball() -> role.ConstraintFn:
    """Creates a constraint function that returns true if the current robot has the
    ball.
    :return: Constraint function for current robot having the ball.
    """

    def constraint_fn(
        robot: rc.Robot,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ) -> bool:
        # TODO: Replace this with an evaluation function that determines possession
        return False

    return constraint_fn
