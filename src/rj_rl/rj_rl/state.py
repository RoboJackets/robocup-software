"""State encoding and decoding for the RL observation space.

Converts the raw simulation state (robot positions, ball state, etc.) into
a normalized numpy array suitable for neural network input, and decodes
network outputs back into simulation-level actions.

Normalization uses field dimensions from the shared ``constants`` module
(mirroring ``rj_constants/constants.hpp``) rather than duplicating them.
"""
import numpy as np

from . import constants


class StateEncoder:
    """Encodes the simulation state into a flat observation vector.

    The observation vector layout (per controlled robot):
        [0:2]   - robot position (x, y) normalized to [-1, 1]
        [2:4]   - robot velocity (vx, vy) normalized
        [4:6]   - robot heading (cos θ, sin θ)
        [6:8]   - ball position (x, y) normalized
        [8:10]  - ball velocity (vx, vy) normalized
        [10:12] - own goal center (x, y) normalized
        [12:14] - opponent goal center (x, y) normalized
        [14:16N+14] - teammate positions (x, y) per teammate (normalized)
        [cont.]     - opponent positions (x, y) per opponent (normalized)

    Normalization uses field dimensions from ``constants`` (matching
    ``rj_constants/constants.hpp``).

    Args:
        num_teammates: Number of teammate robots (excluding the controlled one).
        num_opponents: Number of opponent robots.
    """

    def __init__(
        self,
        num_teammates: int = 5,
        num_opponents: int = 6,
    ):
        self.num_teammates = num_teammates
        self.num_opponents = num_opponents

        # Normalization factors from shared constants
        self._half_length = constants.HALF_FIELD_LENGTH
        self._half_width = constants.HALF_FIELD_WIDTH
        self._max_speed = constants.SIM_ROBOT_MAX_KICK_SPEED  # generous upper bound

    @property
    def observation_size(self) -> int:
        """Total size of the observation vector."""
        base = 14  # robot(6) + ball(4) + goals(4)
        teammates = self.num_teammates * 2
        opponents = self.num_opponents * 2
        return base + teammates + opponents

    def encode(
        self,
        robot_pos: np.ndarray,
        robot_vel: np.ndarray,
        robot_heading: float,
        ball_pos: np.ndarray,
        ball_vel: np.ndarray,
        teammate_positions: np.ndarray,
        opponent_positions: np.ndarray,
        own_goal_x: float,
        opp_goal_x: float,
    ) -> np.ndarray:
        """Encode the game state into a normalized observation vector.

        Args:
            robot_pos: (2,) array of robot (x, y) in meters.
            robot_vel: (2,) array of robot (vx, vy) in m/s.
            robot_heading: Robot heading angle in radians.
            ball_pos: (2,) array of ball (x, y) in meters.
            ball_vel: (2,) array of ball (vx, vy) in m/s.
            teammate_positions: (N, 2) array of teammate positions.
            opponent_positions: (M, 2) array of opponent positions.
            own_goal_x: x-coordinate of own goal center.
            opp_goal_x: x-coordinate of opponent goal center.

        Returns:
            Flat numpy array of normalized observations.
        """
        obs = np.zeros(self.observation_size, dtype=np.float32)
        idx = 0

        # Robot state
        obs[idx:idx + 2] = self._normalize_pos(robot_pos)
        idx += 2
        obs[idx:idx + 2] = self._normalize_vel(robot_vel)
        idx += 2
        obs[idx] = np.cos(robot_heading)
        obs[idx + 1] = np.sin(robot_heading)
        idx += 2

        # Ball state
        obs[idx:idx + 2] = self._normalize_pos(ball_pos)
        idx += 2
        obs[idx:idx + 2] = self._normalize_vel(ball_vel)
        idx += 2

        # Goal positions
        obs[idx:idx + 2] = self._normalize_pos(
            np.array([own_goal_x, 0.0])
        )
        idx += 2
        obs[idx:idx + 2] = self._normalize_pos(
            np.array([opp_goal_x, 0.0])
        )
        idx += 2

        # Teammate positions
        n_tm = min(len(teammate_positions), self.num_teammates)
        for i in range(n_tm):
            obs[idx:idx + 2] = self._normalize_pos(teammate_positions[i])
            idx += 2
        idx += (self.num_teammates - n_tm) * 2  # skip unused slots (zeros)

        # Opponent positions
        n_op = min(len(opponent_positions), self.num_opponents)
        for i in range(n_op):
            obs[idx:idx + 2] = self._normalize_pos(opponent_positions[i])
            idx += 2

        return obs

    def _normalize_pos(self, pos: np.ndarray) -> np.ndarray:
        """Normalize position to approximately [-1, 1]."""
        return np.array(
            [pos[0] / self._half_length, pos[1] / self._half_width],
            dtype=np.float32,
        )

    def _normalize_vel(self, vel: np.ndarray) -> np.ndarray:
        """Normalize velocity to approximately [-1, 1]."""
        return np.array(vel, dtype=np.float32) / self._max_speed
