"""Tests for state encoding."""
import numpy as np
import pytest

from rj_rl import constants
from rj_rl.state import StateEncoder


class TestStateEncoder:
    """Tests for the StateEncoder class."""

    def setup_method(self):
        self.encoder = StateEncoder(num_teammates=5, num_opponents=6)

    def test_observation_size(self):
        # 14 base + 5*2 teammates + 6*2 opponents = 36
        assert self.encoder.observation_size == 36

    def test_encode_returns_correct_shape(self):
        obs = self.encoder.encode(
            robot_pos=np.array([0.0, 0.0]),
            robot_vel=np.array([1.0, 0.0]),
            robot_heading=0.0,
            ball_pos=np.array([1.0, 0.5]),
            ball_vel=np.array([0.0, 0.0]),
            teammate_positions=np.zeros((5, 2)),
            opponent_positions=np.zeros((6, 2)),
            own_goal_x=-4.5,
            opp_goal_x=4.5,
        )
        assert obs.shape == (36,)
        assert obs.dtype == np.float32

    def test_encode_values_normalized(self):
        obs = self.encoder.encode(
            robot_pos=np.array([4.5, 3.0]),  # corner of field
            robot_vel=np.array([3.0, 3.0]),
            robot_heading=np.pi / 4,
            ball_pos=np.array([0.0, 0.0]),
            ball_vel=np.array([0.0, 0.0]),
            teammate_positions=np.zeros((5, 2)),
            opponent_positions=np.zeros((6, 2)),
            own_goal_x=-4.5,
            opp_goal_x=4.5,
        )
        # All values should be roughly in [-1, 1]
        assert np.all(np.abs(obs) <= 2.0)

    def test_encode_center_position_is_zero(self):
        obs = self.encoder.encode(
            robot_pos=np.array([0.0, 0.0]),
            robot_vel=np.array([0.0, 0.0]),
            robot_heading=0.0,
            ball_pos=np.array([0.0, 0.0]),
            ball_vel=np.array([0.0, 0.0]),
            teammate_positions=np.zeros((5, 2)),
            opponent_positions=np.zeros((6, 2)),
            own_goal_x=0.0,
            opp_goal_x=0.0,
        )
        # Position and velocity should be zero
        assert obs[0] == 0.0  # robot x
        assert obs[1] == 0.0  # robot y
        assert obs[2] == 0.0  # robot vx
        assert obs[3] == 0.0  # robot vy
        # heading: cos(0)=1, sin(0)=0
        assert np.isclose(obs[4], 1.0)
        assert np.isclose(obs[5], 0.0)

    def test_fewer_teammates_than_expected(self):
        """Encoder should handle fewer teammates gracefully."""
        obs = self.encoder.encode(
            robot_pos=np.array([0.0, 0.0]),
            robot_vel=np.array([0.0, 0.0]),
            robot_heading=0.0,
            ball_pos=np.array([0.0, 0.0]),
            ball_vel=np.array([0.0, 0.0]),
            teammate_positions=np.zeros((2, 2)),  # only 2 of 5
            opponent_positions=np.zeros((6, 2)),
            own_goal_x=-4.5,
            opp_goal_x=4.5,
        )
        assert obs.shape == (36,)
        # Unused teammate slots should be zero
        assert obs[18] == 0.0  # 3rd teammate x
        assert obs[19] == 0.0  # 3rd teammate y

    def test_custom_num_agents(self):
        encoder = StateEncoder(num_teammates=2, num_opponents=3)
        assert encoder.observation_size == 14 + 2 * 2 + 3 * 2  # 24

    def test_uses_shared_constants(self):
        """Normalization uses constants from the shared module."""
        assert self.encoder._half_length == constants.HALF_FIELD_LENGTH
        assert self.encoder._half_width == constants.HALF_FIELD_WIDTH
