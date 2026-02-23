"""Simple feedforward neural network implemented with numpy.

Provides a lightweight MLP that can be used as a policy or value network
without requiring external deep learning frameworks. Weights are stored
as numpy arrays, making the model easy to save, load, and inspect.
"""
from typing import List, Optional, Tuple

import numpy as np


def _glorot_uniform(fan_in: int, fan_out: int, rng: np.random.Generator) -> np.ndarray:
    """Glorot/Xavier uniform initialization."""
    limit = np.sqrt(6.0 / (fan_in + fan_out))
    return rng.uniform(-limit, limit, size=(fan_in, fan_out)).astype(np.float32)


class NumpyMLP:
    """Multi-layer perceptron implemented in pure numpy.

    Supports forward pass, backpropagation, and gradient-based updates.
    Uses ReLU activations for hidden layers.

    Args:
        layer_sizes: List of layer sizes [input, hidden1, ..., output].
        seed: Random seed for weight initialization.
    """

    def __init__(self, layer_sizes: List[int], seed: int = 42):
        self.rng = np.random.default_rng(seed)
        self.weights: List[np.ndarray] = []
        self.biases: List[np.ndarray] = []

        for i in range(len(layer_sizes) - 1):
            w = _glorot_uniform(layer_sizes[i], layer_sizes[i + 1], self.rng)
            b = np.zeros(layer_sizes[i + 1], dtype=np.float32)
            self.weights.append(w)
            self.biases.append(b)

    def forward(self, x: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """Forward pass through the network.

        Args:
            x: Input array of shape (batch, input_size) or (input_size,).

        Returns:
            Tuple of (output, activations_cache) where activations_cache
            stores pre- and post-activation values for backpropagation.
        """
        single = x.ndim == 1
        if single:
            x = x.reshape(1, -1)

        cache = [x]
        h = x
        for i, (w, b) in enumerate(zip(self.weights, self.biases)):
            z = h @ w + b
            cache.append(z)
            if i < len(self.weights) - 1:
                h = np.maximum(z, 0)  # ReLU
                cache.append(h)
            else:
                h = z  # linear output

        if single:
            h = h.squeeze(0)
        return h, cache

    def get_params(self) -> List[np.ndarray]:
        """Return a flat list of all parameters (weights and biases)."""
        params = []
        for w, b in zip(self.weights, self.biases):
            params.extend([w, b])
        return params

    def set_params(self, params: List[np.ndarray]) -> None:
        """Set all parameters from a flat list."""
        idx = 0
        for i in range(len(self.weights)):
            self.weights[i] = params[idx].copy()
            self.biases[i] = params[idx + 1].copy()
            idx += 2

    def save(self, path: str) -> None:
        """Save model parameters to a .npz file."""
        save_dict = {}
        for i, (w, b) in enumerate(zip(self.weights, self.biases)):
            save_dict[f"w{i}"] = w
            save_dict[f"b{i}"] = b
        np.savez(path, **save_dict)

    def load(self, path: str) -> None:
        """Load model parameters from a .npz file."""
        data = np.load(path)
        for i in range(len(self.weights)):
            self.weights[i] = data[f"w{i}"]
            self.biases[i] = data[f"b{i}"]


class PolicyNetwork:
    """Actor-Critic policy network using two separate MLPs.

    The actor outputs action logits (for discrete actions) and the critic
    outputs a scalar state value estimate.

    Args:
        obs_size: Dimension of the observation vector.
        action_size: Number of discrete actions.
        hidden_sizes: Sizes of hidden layers.
        seed: Random seed.
    """

    def __init__(
        self,
        obs_size: int,
        action_size: int,
        hidden_sizes: Optional[List[int]] = None,
        seed: int = 42,
    ):
        if hidden_sizes is None:
            hidden_sizes = [64, 64]

        self.obs_size = obs_size
        self.action_size = action_size

        actor_sizes = [obs_size] + hidden_sizes + [action_size]
        critic_sizes = [obs_size] + hidden_sizes + [1]

        self.actor = NumpyMLP(actor_sizes, seed=seed)
        self.critic = NumpyMLP(critic_sizes, seed=seed + 1)

    def get_action_and_value(
        self, obs: np.ndarray, deterministic: bool = False
    ) -> Tuple[int, float, np.ndarray, float]:
        """Select an action and compute value estimate.

        Args:
            obs: Observation vector.
            deterministic: If True, select the highest-probability action.

        Returns:
            Tuple of (action, log_prob, action_probs, value).
        """
        logits, _ = self.actor.forward(obs)
        value_out, _ = self.critic.forward(obs)

        # Stable softmax
        logits_shifted = logits - np.max(logits)
        exp_logits = np.exp(logits_shifted)
        probs = exp_logits / np.sum(exp_logits)
        probs = np.clip(probs, 1e-8, 1.0)
        probs = probs / np.sum(probs)

        if deterministic:
            action = int(np.argmax(probs))
        else:
            action = int(np.random.choice(len(probs), p=probs))

        log_prob = float(np.log(probs[action]))
        value = float(value_out.item() if value_out.ndim > 0 else value_out)

        return action, log_prob, probs, value

    def get_value(self, obs: np.ndarray) -> float:
        """Compute state value estimate."""
        value_out, _ = self.critic.forward(obs)
        return float(value_out.item() if value_out.ndim > 0 else value_out)

    def get_action_probs(self, obs: np.ndarray) -> np.ndarray:
        """Compute action probability distribution."""
        logits, _ = self.actor.forward(obs)
        logits_shifted = logits - np.max(logits)
        exp_logits = np.exp(logits_shifted)
        probs = exp_logits / np.sum(exp_logits)
        return np.clip(probs, 1e-8, 1.0)

    def save(self, path: str) -> None:
        """Save both actor and critic networks."""
        self.actor.save(f"{path}_actor.npz")
        self.critic.save(f"{path}_critic.npz")

    def load(self, path: str) -> None:
        """Load both actor and critic networks."""
        self.actor.load(f"{path}_actor.npz")
        self.critic.load(f"{path}_critic.npz")
