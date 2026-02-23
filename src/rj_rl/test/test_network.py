"""Tests for the neural network implementation."""
import numpy as np
import pytest

from rj_rl.network import NumpyMLP, PolicyNetwork


class TestNumpyMLP:
    """Tests for the NumpyMLP class."""

    def test_construction(self):
        mlp = NumpyMLP([10, 32, 5])
        assert len(mlp.weights) == 2
        assert len(mlp.biases) == 2
        assert mlp.weights[0].shape == (10, 32)
        assert mlp.weights[1].shape == (32, 5)

    def test_forward_single(self):
        mlp = NumpyMLP([4, 8, 3], seed=42)
        x = np.ones(4, dtype=np.float32)
        output, cache = mlp.forward(x)
        assert output.shape == (3,)
        assert np.all(np.isfinite(output))

    def test_forward_batch(self):
        mlp = NumpyMLP([4, 8, 3], seed=42)
        x = np.ones((5, 4), dtype=np.float32)
        output, cache = mlp.forward(x)
        assert output.shape == (5, 3)
        assert np.all(np.isfinite(output))

    def test_deterministic_with_same_seed(self):
        mlp1 = NumpyMLP([4, 8, 3], seed=42)
        mlp2 = NumpyMLP([4, 8, 3], seed=42)
        x = np.ones(4, dtype=np.float32)
        out1, _ = mlp1.forward(x)
        out2, _ = mlp2.forward(x)
        np.testing.assert_array_equal(out1, out2)

    def test_get_set_params(self):
        mlp = NumpyMLP([4, 8, 3], seed=42)
        params = mlp.get_params()
        assert len(params) == 4  # 2 weights + 2 biases

        # Modify and restore
        for p in params:
            p *= 2.0
        mlp.set_params(params)
        new_params = mlp.get_params()
        for orig, new in zip(params, new_params):
            np.testing.assert_array_equal(orig, new)

    def test_save_and_load(self, tmp_path):
        mlp = NumpyMLP([4, 8, 3], seed=42)
        x = np.ones(4, dtype=np.float32)
        out_before, _ = mlp.forward(x)

        path = str(tmp_path / "test_model.npz")
        mlp.save(path)

        mlp2 = NumpyMLP([4, 8, 3], seed=99)  # different seed
        mlp2.load(path)
        out_after, _ = mlp2.forward(x)

        np.testing.assert_array_almost_equal(out_before, out_after)


class TestPolicyNetwork:
    """Tests for the PolicyNetwork class."""

    def test_construction(self):
        policy = PolicyNetwork(obs_size=10, action_size=7, hidden_sizes=[32, 32])
        assert policy.obs_size == 10
        assert policy.action_size == 7

    def test_get_action_and_value(self):
        policy = PolicyNetwork(obs_size=10, action_size=7, seed=42)
        obs = np.random.randn(10).astype(np.float32)
        action, log_prob, probs, value = policy.get_action_and_value(obs)

        assert isinstance(action, int)
        assert 0 <= action < 7
        assert isinstance(log_prob, float)
        assert log_prob <= 0  # log probabilities are non-positive
        assert probs.shape == (7,)
        assert np.isclose(np.sum(probs), 1.0, atol=1e-5)
        assert isinstance(value, float)

    def test_deterministic_action(self):
        policy = PolicyNetwork(obs_size=10, action_size=7, seed=42)
        obs = np.random.randn(10).astype(np.float32)

        actions = set()
        for _ in range(20):
            action, _, _, _ = policy.get_action_and_value(obs, deterministic=True)
            actions.add(action)

        assert len(actions) == 1  # deterministic should always return same action

    def test_get_value(self):
        policy = PolicyNetwork(obs_size=10, action_size=7, seed=42)
        obs = np.random.randn(10).astype(np.float32)
        value = policy.get_value(obs)
        assert isinstance(value, float)
        assert np.isfinite(value)

    def test_get_action_probs_sum_to_one(self):
        policy = PolicyNetwork(obs_size=10, action_size=7, seed=42)
        obs = np.random.randn(10).astype(np.float32)
        probs = policy.get_action_probs(obs)
        assert np.isclose(np.sum(probs), 1.0, atol=1e-5)
        assert np.all(probs >= 0)

    def test_save_and_load(self, tmp_path):
        policy = PolicyNetwork(obs_size=10, action_size=7, seed=42)
        obs = np.random.randn(10).astype(np.float32)

        _, _, probs_before, value_before = policy.get_action_and_value(obs)

        path = str(tmp_path / "test_policy")
        policy.save(path)

        policy2 = PolicyNetwork(obs_size=10, action_size=7, seed=99)
        policy2.load(path)

        _, _, probs_after, value_after = policy2.get_action_and_value(obs)

        np.testing.assert_array_almost_equal(probs_before, probs_after)
        assert np.isclose(value_before, value_after, atol=1e-5)
