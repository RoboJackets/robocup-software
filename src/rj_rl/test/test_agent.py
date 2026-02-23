"""Tests for the PPO agent."""
import numpy as np
import pytest

from rj_rl.agent import PPOAgent, RolloutBuffer, compute_gae
from rj_rl.config import TrainingConfig


class TestRolloutBuffer:
    """Tests for the RolloutBuffer class."""

    def test_add_and_size(self):
        buf = RolloutBuffer()
        assert buf.size == 0
        buf.add(np.zeros(4), 0, 1.0, 0.5, -0.5, False)
        assert buf.size == 1
        buf.add(np.ones(4), 1, -1.0, 0.3, -1.0, True)
        assert buf.size == 2

    def test_clear(self):
        buf = RolloutBuffer()
        buf.add(np.zeros(4), 0, 1.0, 0.5, -0.5, False)
        buf.clear()
        assert buf.size == 0

    def test_stores_copy_of_observation(self):
        buf = RolloutBuffer()
        obs = np.array([1.0, 2.0, 3.0])
        buf.add(obs, 0, 0.0, 0.0, 0.0, False)
        obs[0] = 999.0
        assert buf.observations[0][0] == 1.0  # should not be modified


class TestComputeGAE:
    """Tests for GAE computation."""

    def test_single_step_no_discount(self):
        rewards = np.array([1.0])
        values = np.array([0.5])
        dones = np.array([True])
        advantages = compute_gae(rewards, values, dones, 0.0, gamma=0.99, gae_lambda=0.95)
        # delta = r + gamma * next_value * (1 - done) - V = 1.0 + 0 - 0.5 = 0.5
        assert np.isclose(advantages[0], 0.5)

    def test_multi_step_advantages(self):
        rewards = np.array([1.0, 2.0, 3.0])
        values = np.array([0.0, 0.0, 0.0])
        dones = np.array([False, False, True])
        advantages = compute_gae(rewards, values, dones, 0.0, gamma=1.0, gae_lambda=1.0)
        # With gamma=1, lambda=1, and V=0 everywhere:
        # A[2] = 3.0
        # A[1] = 2.0 + 1.0 * 3.0 = 5.0
        # A[0] = 1.0 + 1.0 * 5.0 = 6.0
        np.testing.assert_array_almost_equal(advantages, [6.0, 5.0, 3.0])

    def test_done_resets_gae(self):
        rewards = np.array([1.0, 2.0])
        values = np.array([0.0, 0.0])
        dones = np.array([True, False])
        advantages = compute_gae(rewards, values, dones, 1.0, gamma=0.99, gae_lambda=0.95)
        # Step 0 done=True: delta = 1.0 + 0 - 0 = 1.0, gae = 1.0
        # Step 1 done=False: delta = 2.0 + 0.99*1.0 - 0 = 2.99, gae = 2.99
        assert np.isclose(advantages[0], 1.0)
        assert np.isclose(advantages[1], 2.99)


class TestPPOAgent:
    """Tests for the PPOAgent class."""

    def setup_method(self):
        self.config = TrainingConfig(
            batch_size=4,
            n_epochs=1,
            rollout_steps=8,
            hidden_sizes=[16, 16],
        )
        self.agent = PPOAgent(obs_size=10, action_size=7, config=self.config)

    def test_select_action(self):
        obs = np.random.randn(10).astype(np.float32)
        action, log_prob, value = self.agent.select_action(obs)
        assert isinstance(action, int)
        assert 0 <= action < 7
        assert isinstance(log_prob, float)
        assert isinstance(value, float)

    def test_select_action_deterministic(self):
        obs = np.random.randn(10).astype(np.float32)
        actions = set()
        for _ in range(20):
            action, _, _ = self.agent.select_action(obs, deterministic=True)
            actions.add(action)
        assert len(actions) == 1

    def test_update_with_data(self):
        obs = np.random.randn(10).astype(np.float32)
        for _ in range(8):
            action, log_prob, value = self.agent.select_action(obs)
            self.agent.buffer.add(obs, action, 1.0, value, log_prob, False)
        metrics = self.agent.update(obs)
        assert "policy_loss" in metrics
        assert "value_loss" in metrics
        assert "entropy" in metrics

    def test_update_empty_buffer_returns_empty(self):
        obs = np.random.randn(10).astype(np.float32)
        metrics = self.agent.update(obs)
        assert metrics == {}

    def test_save_and_load(self, tmp_path):
        obs = np.random.randn(10).astype(np.float32)
        _, _, probs_before, _ = self.agent.policy.get_action_and_value(obs)

        path = str(tmp_path / "test_agent")
        self.agent.save(path)

        new_agent = PPOAgent(obs_size=10, action_size=7, config=self.config)
        new_agent.load(path)
        _, _, probs_after, _ = new_agent.policy.get_action_and_value(obs)

        np.testing.assert_array_almost_equal(probs_before, probs_after)
