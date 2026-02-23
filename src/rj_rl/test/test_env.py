"""Tests for the RoboCup RL environment."""
import numpy as np
import pytest

from rj_rl.config import RLConfig
from rj_rl.env import RoboCupEnv
from rj_rl.action import ActionType


class TestRoboCupEnvInit:
    """Tests for environment initialization."""

    def test_default_construction(self):
        env = RoboCupEnv()
        assert env.observation_size > 0
        assert env.action_size == ActionType.size()

    def test_custom_config(self):
        config = RLConfig()
        config.env.num_blue_robots = 3
        config.env.num_yellow_robots = 3
        env = RoboCupEnv(config)
        assert len(env.blue_robots) == 0  # not initialized until reset
        obs = env.reset()
        assert len(env.blue_robots) == 3
        assert len(env.yellow_robots) == 3


class TestRoboCupEnvReset:
    """Tests for environment reset."""

    def test_reset_returns_observation(self):
        env = RoboCupEnv()
        obs = env.reset()
        assert isinstance(obs, np.ndarray)
        assert obs.shape == (env.observation_size,)
        assert np.all(np.isfinite(obs))

    def test_reset_deterministic_with_seed(self):
        env = RoboCupEnv()
        obs1 = env.reset(seed=42)
        obs2 = env.reset(seed=42)
        np.testing.assert_array_equal(obs1, obs2)

    def test_reset_different_seeds_differ(self):
        env = RoboCupEnv()
        obs1 = env.reset(seed=42)
        obs2 = env.reset(seed=123)
        assert not np.array_equal(obs1, obs2)

    def test_reset_clears_step_count(self):
        env = RoboCupEnv()
        env.reset()
        env.step(ActionType.IDLE)
        assert env.step_count == 1
        env.reset()
        assert env.step_count == 0


class TestRoboCupEnvStep:
    """Tests for environment stepping."""

    def test_step_returns_correct_types(self):
        env = RoboCupEnv()
        env.reset(seed=42)
        obs, reward, done, info = env.step(ActionType.IDLE)
        assert isinstance(obs, np.ndarray)
        assert isinstance(reward, (float, np.floating))
        assert isinstance(done, bool)
        assert isinstance(info, dict)

    def test_step_observation_shape(self):
        env = RoboCupEnv()
        env.reset(seed=42)
        obs, _, _, _ = env.step(ActionType.MOVE_TO_BALL)
        assert obs.shape == (env.observation_size,)
        assert np.all(np.isfinite(obs))

    def test_step_increments_count(self):
        env = RoboCupEnv()
        env.reset()
        for i in range(5):
            env.step(ActionType.IDLE)
        assert env.step_count == 5

    def test_all_actions_are_valid(self):
        env = RoboCupEnv()
        env.reset(seed=42)
        for action in ActionType:
            obs, reward, done, info = env.step(int(action))
            assert np.all(np.isfinite(obs))
            if done:
                env.reset(seed=42)

    def test_episode_ends_on_max_steps(self):
        config = RLConfig()
        config.env.max_episode_steps = 10
        env = RoboCupEnv(config)
        env.reset()
        done = False
        for _ in range(10):
            _, _, done, info = env.step(ActionType.IDLE)
        assert done
        assert info.get("result") == "timeout"


class TestRoboCupEnvPhysics:
    """Tests for environment physics."""

    def test_ball_position_changes_when_kicked(self):
        env = RoboCupEnv()
        env.reset(seed=42)
        initial_ball = env.ball.pos.copy()
        # Move robot to ball and kick
        for _ in range(200):
            env.step(ActionType.SHOOT_ON_GOAL)
        # Ball should have moved
        assert not np.allclose(env.ball.pos, initial_ball, atol=0.1)

    def test_robots_stay_in_bounds(self):
        env = RoboCupEnv()
        env.reset(seed=42)
        half_l = env._half_length
        half_w = env._half_width
        for _ in range(100):
            env.step(ActionType.POSITION_OFFENSE)
        for robot in env.blue_robots + env.yellow_robots:
            assert abs(robot.pos[0]) <= half_l + 0.01
            assert abs(robot.pos[1]) <= half_w + 0.01

    def test_ball_stays_in_bounds_or_scores(self):
        env = RoboCupEnv()
        env.reset(seed=42)
        for _ in range(500):
            _, _, done, _ = env.step(ActionType.SHOOT_ON_GOAL)
            if done:
                break
        # Ball should be within field bounds (or a goal was scored)
        if not done:
            assert abs(env.ball.pos[1]) <= env._half_width + 0.1
