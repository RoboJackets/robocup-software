"""Reward functions for the RL environment.

Provides composable reward components that can be weighted and combined
to guide learning. Each reward component is a pure function that takes
the current and previous state and returns a scalar reward. Physical
constants come from the shared ``constants`` module.
"""
import numpy as np

from . import constants
from .config import RewardConfig


class RewardComputer:
    """Computes the total reward from individual reward components.

    The total reward is a weighted sum of:
        - Goal scored / conceded
        - Ball possession (proximity to ball)
        - Ball progress toward opponent goal
        - Time penalty (encourages faster play)

    Args:
        config: Reward weights configuration.
        opp_goal_x: x-coordinate of the opponent's goal center.
        field_length: Length of the field in meters.
    """

    def __init__(
        self,
        config: RewardConfig,
        opp_goal_x: float,
        field_length: float,
    ):
        self.config = config
        self.opp_goal_x = opp_goal_x
        self.field_length = field_length

    def compute(
        self,
        ball_pos: np.ndarray,
        prev_ball_pos: np.ndarray,
        robot_positions: np.ndarray,
        goal_scored: bool,
        goal_conceded: bool,
    ) -> float:
        """Compute total reward for one timestep.

        Args:
            ball_pos: Current ball (x, y).
            prev_ball_pos: Previous ball (x, y).
            robot_positions: (N, 2) positions of controlled team's robots.
            goal_scored: True if the controlled team scored this step.
            goal_conceded: True if the opponent scored this step.

        Returns:
            Scalar reward value.
        """
        reward = 0.0

        # Goal events
        if goal_scored:
            reward += self.config.goal_scored
        if goal_conceded:
            reward += self.config.goal_conceded

        # Ball possession: reward for having a robot close to the ball
        reward += self._ball_possession_reward(ball_pos, robot_positions)

        # Ball progress toward opponent goal
        reward += self._ball_progress_reward(ball_pos, prev_ball_pos)

        # Time penalty
        reward += self.config.time_penalty

        return reward

    def _ball_possession_reward(
        self,
        ball_pos: np.ndarray,
        robot_positions: np.ndarray,
    ) -> float:
        """Reward for having a robot close to the ball."""
        if len(robot_positions) == 0:
            return 0.0

        distances = np.linalg.norm(robot_positions - ball_pos, axis=1)
        min_dist = np.min(distances)

        # Reward inversely proportional to distance, capped at robot radius
        possession_threshold = constants.ROBOT_RADIUS + constants.BALL_RADIUS
        if min_dist < possession_threshold:
            return self.config.ball_possession
        return self.config.ball_possession * max(
            0.0, 1.0 - min_dist / 2.0
        )

    def _ball_progress_reward(
        self,
        ball_pos: np.ndarray,
        prev_ball_pos: np.ndarray,
    ) -> float:
        """Reward for moving the ball toward the opponent's goal."""
        prev_dist = abs(prev_ball_pos[0] - self.opp_goal_x)
        curr_dist = abs(ball_pos[0] - self.opp_goal_x)
        progress = prev_dist - curr_dist  # positive = closer to goal

        return self.config.ball_progress * progress
