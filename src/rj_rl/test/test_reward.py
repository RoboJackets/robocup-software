"""Tests for reward computation."""
import numpy as np
import pytest

from rj_rl.config import RewardConfig
from rj_rl.reward import RewardComputer


class TestRewardComputer:
    """Tests for the RewardComputer class."""

    def setup_method(self):
        self.config = RewardConfig()
        self.computer = RewardComputer(
            config=self.config, opp_goal_x=4.5, field_length=9.0
        )

    def test_goal_scored_gives_positive_reward(self):
        reward = self.computer.compute(
            ball_pos=np.array([4.5, 0.0]),
            prev_ball_pos=np.array([4.0, 0.0]),
            robot_positions=np.array([[1.0, 0.0]]),
            goal_scored=True,
            goal_conceded=False,
        )
        assert reward > 5.0  # Should be dominated by goal reward

    def test_goal_conceded_gives_negative_reward(self):
        reward = self.computer.compute(
            ball_pos=np.array([-4.5, 0.0]),
            prev_ball_pos=np.array([-4.0, 0.0]),
            robot_positions=np.array([[1.0, 0.0]]),
            goal_scored=False,
            goal_conceded=True,
        )
        assert reward < -5.0

    def test_ball_possession_reward(self):
        # Robot close to ball → higher reward
        reward_close = self.computer.compute(
            ball_pos=np.array([1.0, 0.0]),
            prev_ball_pos=np.array([1.0, 0.0]),
            robot_positions=np.array([[1.05, 0.0]]),
            goal_scored=False,
            goal_conceded=False,
        )
        reward_far = self.computer.compute(
            ball_pos=np.array([1.0, 0.0]),
            prev_ball_pos=np.array([1.0, 0.0]),
            robot_positions=np.array([[4.0, 3.0]]),
            goal_scored=False,
            goal_conceded=False,
        )
        assert reward_close > reward_far

    def test_ball_progress_reward(self):
        # Ball moving toward opponent goal → positive progress
        reward_forward = self.computer.compute(
            ball_pos=np.array([2.0, 0.0]),
            prev_ball_pos=np.array([1.0, 0.0]),
            robot_positions=np.array([[5.0, 5.0]]),  # far from ball
            goal_scored=False,
            goal_conceded=False,
        )
        reward_backward = self.computer.compute(
            ball_pos=np.array([0.0, 0.0]),
            prev_ball_pos=np.array([1.0, 0.0]),
            robot_positions=np.array([[5.0, 5.0]]),  # far from ball
            goal_scored=False,
            goal_conceded=False,
        )
        assert reward_forward > reward_backward

    def test_time_penalty_is_negative(self):
        reward = self.computer.compute(
            ball_pos=np.array([0.0, 0.0]),
            prev_ball_pos=np.array([0.0, 0.0]),
            robot_positions=np.array([[5.0, 5.0]]),
            goal_scored=False,
            goal_conceded=False,
        )
        # With no events, time penalty should make reward slightly negative
        assert reward < 0.0

    def test_empty_robot_positions(self):
        reward = self.computer.compute(
            ball_pos=np.array([0.0, 0.0]),
            prev_ball_pos=np.array([0.0, 0.0]),
            robot_positions=np.empty((0, 2)),
            goal_scored=False,
            goal_conceded=False,
        )
        assert np.isfinite(reward)

    def test_custom_weights(self):
        config = RewardConfig(
            goal_scored=100.0,
            goal_conceded=-100.0,
            ball_possession=0.0,
            ball_progress=0.0,
            time_penalty=0.0,
        )
        computer = RewardComputer(config, opp_goal_x=4.5, field_length=9.0)
        reward = computer.compute(
            ball_pos=np.array([0.0, 0.0]),
            prev_ball_pos=np.array([0.0, 0.0]),
            robot_positions=np.array([[0.0, 0.0]]),
            goal_scored=True,
            goal_conceded=False,
        )
        assert reward == 100.0
