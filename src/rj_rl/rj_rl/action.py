"""Action space definitions for the RL agent.

Defines the discrete action set available to each controlled robot and
provides utilities for converting between action indices and simulation
commands.
"""
from enum import IntEnum
from typing import Tuple

import numpy as np


class ActionType(IntEnum):
    """Discrete action set for a single robot.

    Each action maps to a high-level behavior that the environment
    translates into low-level motion commands.
    """

    MOVE_TO_BALL = 0
    SHOOT_ON_GOAL = 1
    PASS_TO_NEAREST_TEAMMATE = 2
    DEFEND_GOAL = 3
    POSITION_OFFENSE = 4
    POSITION_DEFENSE = 5
    IDLE = 6

    @classmethod
    def size(cls) -> int:
        """Number of discrete actions."""
        return len(cls)


def action_to_target(
    action: int,
    robot_pos: np.ndarray,
    ball_pos: np.ndarray,
    teammate_positions: np.ndarray,
    own_goal_pos: np.ndarray,
    opp_goal_pos: np.ndarray,
    field_length: float,
    field_width: float,
) -> Tuple[np.ndarray, bool]:
    """Convert a discrete action index to a target position and kick flag.

    Args:
        action: Action index (see ActionType).
        robot_pos: Current robot (x, y).
        ball_pos: Current ball (x, y).
        teammate_positions: (N, 2) array of teammate positions.
        own_goal_pos: (2,) own goal center position.
        opp_goal_pos: (2,) opponent goal center position.
        field_length: Field length in meters.
        field_width: Field width in meters.

    Returns:
        Tuple of (target_position (2,), should_kick: bool).
    """
    should_kick = False

    if action == ActionType.MOVE_TO_BALL:
        target = ball_pos.copy()

    elif action == ActionType.SHOOT_ON_GOAL:
        target = ball_pos.copy()
        should_kick = True

    elif action == ActionType.PASS_TO_NEAREST_TEAMMATE:
        target = ball_pos.copy()
        if len(teammate_positions) > 0:
            should_kick = True

    elif action == ActionType.DEFEND_GOAL:
        # Position between ball and own goal
        direction = ball_pos - own_goal_pos
        dist = np.linalg.norm(direction)
        if dist > 1e-6:
            direction = direction / dist
        target = own_goal_pos + direction * min(dist * 0.3, 1.5)

    elif action == ActionType.POSITION_OFFENSE:
        # Move toward opponent half, offset from ball
        target = np.array(
            [
                opp_goal_pos[0] * 0.5,
                np.clip(ball_pos[1] + 1.0, -field_width / 2, field_width / 2),
            ]
        )

    elif action == ActionType.POSITION_DEFENSE:
        # Stay in defensive half, track ball y-position
        target = np.array(
            [
                own_goal_pos[0] * 0.5,
                np.clip(ball_pos[1], -field_width / 2, field_width / 2),
            ]
        )

    else:  # IDLE
        target = robot_pos.copy()

    return target, should_kick
