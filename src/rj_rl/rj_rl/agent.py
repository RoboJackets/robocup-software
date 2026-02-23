"""PPO (Proximal Policy Optimization) agent implemented with numpy.

Provides a complete PPO implementation that collects experience, computes
advantages using GAE (Generalized Advantage Estimation), and updates the
policy using clipped surrogate objectives - all without requiring external
deep learning frameworks.

References:
    Schulman et al., "Proximal Policy Optimization Algorithms", 2017.
"""
from typing import Dict, List, Optional

import numpy as np

from .config import TrainingConfig
from .network import PolicyNetwork


class RolloutBuffer:
    """Stores experience collected during environment rollouts.

    Manages observations, actions, rewards, values, and log-probabilities
    needed for PPO updates.
    """

    def __init__(self) -> None:
        self.observations: List[np.ndarray] = []
        self.actions: List[int] = []
        self.rewards: List[float] = []
        self.values: List[float] = []
        self.log_probs: List[float] = []
        self.dones: List[bool] = []

    def add(
        self,
        obs: np.ndarray,
        action: int,
        reward: float,
        value: float,
        log_prob: float,
        done: bool,
    ) -> None:
        """Add a single transition to the buffer."""
        self.observations.append(obs.copy())
        self.actions.append(action)
        self.rewards.append(reward)
        self.values.append(value)
        self.log_probs.append(log_prob)
        self.dones.append(done)

    def clear(self) -> None:
        """Clear all stored transitions."""
        self.observations.clear()
        self.actions.clear()
        self.rewards.clear()
        self.values.clear()
        self.log_probs.clear()
        self.dones.clear()

    @property
    def size(self) -> int:
        return len(self.observations)


def compute_gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    last_value: float,
    gamma: float,
    gae_lambda: float,
) -> np.ndarray:
    """Compute Generalized Advantage Estimation.

    Args:
        rewards: (T,) array of rewards.
        values: (T,) array of value estimates.
        dones: (T,) boolean array, True when episode ends.
        last_value: Value estimate for the state after the last step.
        gamma: Discount factor.
        gae_lambda: GAE lambda parameter.

    Returns:
        (T,) array of advantage estimates.
    """
    n = len(rewards)
    advantages = np.zeros(n, dtype=np.float32)
    last_gae = 0.0

    for t in reversed(range(n)):
        if t == n - 1:
            next_value = last_value
            next_non_terminal = 1.0 - float(dones[t])
        else:
            next_value = values[t + 1]
            next_non_terminal = 1.0 - float(dones[t])

        delta = rewards[t] + gamma * next_value * next_non_terminal - values[t]
        last_gae = delta + gamma * gae_lambda * next_non_terminal * last_gae
        advantages[t] = last_gae

    return advantages


class PPOAgent:
    """Proximal Policy Optimization agent.

    Uses a clipped surrogate objective for stable policy updates and
    an actor-critic architecture for variance reduction.

    Args:
        obs_size: Dimension of the observation space.
        action_size: Number of discrete actions.
        config: Training configuration (optional, uses defaults if None).
    """

    def __init__(
        self,
        obs_size: int,
        action_size: int,
        config: Optional[TrainingConfig] = None,
    ):
        if config is None:
            config = TrainingConfig()

        self.config = config
        self.obs_size = obs_size
        self.action_size = action_size

        self.policy = PolicyNetwork(
            obs_size=obs_size,
            action_size=action_size,
            hidden_sizes=config.hidden_sizes,
            seed=config.seed,
        )

        self.buffer = RolloutBuffer()

        # Adam optimizer state
        self._actor_m = [np.zeros_like(p) for p in self.policy.actor.get_params()]
        self._actor_v = [np.zeros_like(p) for p in self.policy.actor.get_params()]
        self._critic_m = [np.zeros_like(p) for p in self.policy.critic.get_params()]
        self._critic_v = [np.zeros_like(p) for p in self.policy.critic.get_params()]
        self._update_step = 0

    def select_action(
        self, obs: np.ndarray, deterministic: bool = False
    ) -> tuple:
        """Select an action given the current observation.

        Args:
            obs: Observation vector.
            deterministic: If True, select greedy action.

        Returns:
            Tuple of (action, log_prob, value).
        """
        action, log_prob, _probs, value = self.policy.get_action_and_value(
            obs, deterministic=deterministic
        )
        return action, log_prob, value

    def update(self, last_obs: np.ndarray) -> Dict[str, float]:
        """Run PPO update using collected experience.

        Args:
            last_obs: Observation after the last step (for bootstrapping).

        Returns:
            Dictionary of training metrics.
        """
        if self.buffer.size == 0:
            return {}

        # Convert buffer to arrays
        observations = np.array(self.buffer.observations)
        actions = np.array(self.buffer.actions)
        old_log_probs = np.array(self.buffer.log_probs)
        rewards = np.array(self.buffer.rewards, dtype=np.float32)
        values = np.array(self.buffer.values, dtype=np.float32)
        dones = np.array(self.buffer.dones, dtype=np.float32)

        # Bootstrap value
        last_value = self.policy.get_value(last_obs)

        # Compute advantages and returns
        advantages = compute_gae(
            rewards, values, dones, last_value,
            self.config.gamma, self.config.gae_lambda,
        )
        returns = advantages + values

        # Normalize advantages
        if len(advantages) > 1:
            adv_std = np.std(advantages)
            if adv_std > 1e-8:
                advantages = (advantages - np.mean(advantages)) / adv_std

        # PPO epochs
        total_policy_loss = 0.0
        total_value_loss = 0.0
        total_entropy = 0.0
        n_updates = 0

        buffer_size = len(observations)
        indices = np.arange(buffer_size)

        for _epoch in range(self.config.n_epochs):
            np.random.shuffle(indices)
            for start in range(0, buffer_size, self.config.batch_size):
                end = min(start + self.config.batch_size, buffer_size)
                batch_idx = indices[start:end]

                batch_obs = observations[batch_idx]
                batch_actions = actions[batch_idx]
                batch_old_log_probs = old_log_probs[batch_idx]
                batch_advantages = advantages[batch_idx]
                batch_returns = returns[batch_idx]

                # Compute current policy log probs and values
                policy_loss, value_loss, entropy = self._compute_losses(
                    batch_obs,
                    batch_actions,
                    batch_old_log_probs,
                    batch_advantages,
                    batch_returns,
                )

                total_policy_loss += policy_loss
                total_value_loss += value_loss
                total_entropy += entropy
                n_updates += 1

        self.buffer.clear()

        n_updates = max(n_updates, 1)
        return {
            "policy_loss": total_policy_loss / n_updates,
            "value_loss": total_value_loss / n_updates,
            "entropy": total_entropy / n_updates,
        }

    def _compute_losses(
        self,
        observations: np.ndarray,
        actions: np.ndarray,
        old_log_probs: np.ndarray,
        advantages: np.ndarray,
        returns: np.ndarray,
    ) -> tuple:
        """Compute PPO losses and update networks using finite differences.

        Uses numerical gradient estimation for the numpy-based networks.
        This is less efficient than backpropagation but keeps the
        implementation simple and dependency-free.
        """
        perturbation = 1e-3
        lr = self.config.learning_rate

        # --- Actor update ---
        actor_params = self.policy.actor.get_params()

        def actor_objective(params):
            self.policy.actor.set_params(params)
            total = 0.0
            total_entropy = 0.0
            for i in range(len(observations)):
                probs = self.policy.get_action_probs(observations[i])
                log_prob = np.log(probs[actions[i]] + 1e-8)
                ratio = np.exp(log_prob - old_log_probs[i])
                clipped = np.clip(
                    ratio,
                    1.0 - self.config.clip_epsilon,
                    1.0 + self.config.clip_epsilon,
                )
                total += min(ratio * advantages[i], clipped * advantages[i])
                total_entropy -= np.sum(probs * np.log(probs + 1e-8))
            return (
                total / len(observations)
                + self.config.entropy_coeff * total_entropy / len(observations)
            )

        # Compute base objective and entropy for logging
        base_obj = actor_objective(actor_params)

        # Estimate gradients and update actor
        new_actor_params = []
        for p_idx, param in enumerate(actor_params):
            grad = np.zeros_like(param)
            flat = param.flatten()
            # Sample a subset of parameters to perturb for efficiency
            n_samples = min(len(flat), 50)
            sample_indices = np.random.choice(len(flat), n_samples, replace=False)
            for idx in sample_indices:
                old_val = flat[idx]
                flat[idx] = old_val + perturbation
                param_plus = param.copy()
                param_plus.flat[idx] = flat[idx]
                params_plus = actor_params.copy()
                params_plus[p_idx] = param_plus
                obj_plus = actor_objective(params_plus)
                grad.flat[idx] = (obj_plus - base_obj) / perturbation * (
                    len(flat) / n_samples
                )
                flat[idx] = old_val

            # Adam update
            self._update_step += 1
            self._actor_m[p_idx] = (
                0.9 * self._actor_m[p_idx] + 0.1 * grad
            )
            self._actor_v[p_idx] = (
                0.999 * self._actor_v[p_idx] + 0.001 * grad ** 2
            )
            m_hat = self._actor_m[p_idx] / (1 - 0.9 ** self._update_step)
            v_hat = self._actor_v[p_idx] / (1 - 0.999 ** self._update_step)
            updated = param + lr * m_hat / (np.sqrt(v_hat) + 1e-8)
            new_actor_params.append(updated)

        self.policy.actor.set_params(new_actor_params)

        # --- Critic update ---
        critic_params = self.policy.critic.get_params()

        def critic_loss(params):
            self.policy.critic.set_params(params)
            total = 0.0
            for i in range(len(observations)):
                value = self.policy.get_value(observations[i])
                total += (value - returns[i]) ** 2
            return total / len(observations)

        base_vloss = critic_loss(critic_params)

        new_critic_params = []
        for p_idx, param in enumerate(critic_params):
            grad = np.zeros_like(param)
            flat = param.flatten()
            n_samples = min(len(flat), 50)
            sample_indices = np.random.choice(len(flat), n_samples, replace=False)
            for idx in sample_indices:
                old_val = flat[idx]
                flat[idx] = old_val + perturbation
                param_plus = param.copy()
                param_plus.flat[idx] = flat[idx]
                params_plus = critic_params.copy()
                params_plus[p_idx] = param_plus
                vloss_plus = critic_loss(params_plus)
                grad.flat[idx] = (vloss_plus - base_vloss) / perturbation * (
                    len(flat) / n_samples
                )
                flat[idx] = old_val

            # Adam update (minimize loss → negative gradient)
            self._critic_m[p_idx] = (
                0.9 * self._critic_m[p_idx] + 0.1 * grad
            )
            self._critic_v[p_idx] = (
                0.999 * self._critic_v[p_idx] + 0.001 * grad ** 2
            )
            m_hat = self._critic_m[p_idx] / (1 - 0.9 ** self._update_step)
            v_hat = self._critic_v[p_idx] / (1 - 0.999 ** self._update_step)
            updated = param - lr * m_hat / (np.sqrt(v_hat) + 1e-8)
            new_critic_params.append(updated)

        self.policy.critic.set_params(new_critic_params)

        # Compute entropy for logging
        entropy_sum = 0.0
        for i in range(len(observations)):
            probs = self.policy.get_action_probs(observations[i])
            entropy_sum -= np.sum(probs * np.log(probs + 1e-8))

        return (
            -base_obj,
            base_vloss,
            entropy_sum / len(observations),
        )

    def save(self, path: str) -> None:
        """Save the policy network to disk."""
        self.policy.save(path)

    def load(self, path: str) -> None:
        """Load the policy network from disk."""
        self.policy.load(path)
