"""Training loop for the PPO agent.

Orchestrates the collection of experience from the environment, updates
the agent, and logs training metrics. Supports checkpointing for long
training runs.
"""
import os
import time
from typing import Dict, List, Optional

import numpy as np

from .agent import PPOAgent
from .config import RLConfig
from .env import RoboCupEnv


class Trainer:
    """Manages the training loop for the RL agent.

    Collects rollouts from the environment, triggers PPO updates, and
    tracks training metrics.

    Args:
        config: Full RL configuration.
        save_dir: Directory for saving checkpoints (default: "checkpoints").
    """

    def __init__(
        self,
        config: Optional[RLConfig] = None,
        save_dir: str = "checkpoints",
    ):
        if config is None:
            config = RLConfig()

        self.config = config
        self.save_dir = save_dir

        self.env = RoboCupEnv(config)
        self.agent = PPOAgent(
            obs_size=self.env.observation_size,
            action_size=self.env.action_size,
            config=config.training,
        )

        self.total_timesteps = 0
        self.episodes_completed = 0
        self.episode_rewards: List[float] = []
        self.metrics_history: List[Dict[str, float]] = []

    def train(
        self,
        total_timesteps: Optional[int] = None,
        callback=None,
    ) -> Dict[str, List[float]]:
        """Run the full training loop.

        Args:
            total_timesteps: Override total timesteps from config.
            callback: Optional callable(metrics_dict) called each update.

        Returns:
            Dictionary of metric histories.
        """
        if total_timesteps is None:
            total_timesteps = self.config.training.total_timesteps

        os.makedirs(self.save_dir, exist_ok=True)

        obs = self.env.reset()
        episode_reward = 0.0
        iteration = 0

        start_time = time.time()

        while self.total_timesteps < total_timesteps:
            # Collect rollout
            for _step in range(self.config.training.rollout_steps):
                action, log_prob, value = self.agent.select_action(obs)

                next_obs, reward, done, info = self.env.step(action)

                self.agent.buffer.add(obs, action, reward, value, log_prob, done)

                obs = next_obs
                episode_reward += reward
                self.total_timesteps += 1

                if done:
                    self.episode_rewards.append(episode_reward)
                    self.episodes_completed += 1
                    episode_reward = 0.0
                    obs = self.env.reset()

                if self.total_timesteps >= total_timesteps:
                    break

            # PPO update
            metrics = self.agent.update(obs)

            # Logging
            iteration += 1
            elapsed = time.time() - start_time
            fps = self.total_timesteps / max(elapsed, 1e-6)

            recent_rewards = self.episode_rewards[-10:] if self.episode_rewards else [0]
            metrics.update(
                {
                    "mean_reward": float(np.mean(recent_rewards)),
                    "episodes": self.episodes_completed,
                    "timesteps": self.total_timesteps,
                    "fps": fps,
                }
            )
            self.metrics_history.append(metrics)

            if iteration % self.config.training.log_interval == 0:
                print(
                    f"[Iter {iteration}] "
                    f"steps={self.total_timesteps} "
                    f"episodes={self.episodes_completed} "
                    f"mean_reward={metrics['mean_reward']:.3f} "
                    f"fps={fps:.0f}"
                )

            if iteration % self.config.training.save_interval == 0:
                path = os.path.join(
                    self.save_dir, f"policy_iter_{iteration}"
                )
                self.agent.save(path)

            if callback is not None:
                callback(metrics)

        # Final save
        final_path = os.path.join(self.save_dir, "policy_final")
        self.agent.save(final_path)

        return {
            "episode_rewards": self.episode_rewards,
            "metrics": self.metrics_history,
        }

    def evaluate(
        self, n_episodes: int = 10, deterministic: bool = True
    ) -> Dict[str, float]:
        """Evaluate the current policy without training.

        Args:
            n_episodes: Number of evaluation episodes.
            deterministic: Whether to use deterministic action selection.

        Returns:
            Dictionary with mean_reward, std_reward, goals_scored, goals_conceded.
        """
        rewards = []
        goals_scored = 0
        goals_conceded = 0

        for _ in range(n_episodes):
            obs = self.env.reset()
            episode_reward = 0.0
            done = False

            while not done:
                action, _, _ = self.agent.select_action(
                    obs, deterministic=deterministic
                )
                obs, reward, done, info = self.env.step(action)
                episode_reward += reward

                if info.get("result") == "goal_scored":
                    goals_scored += 1
                elif info.get("result") == "goal_conceded":
                    goals_conceded += 1

            rewards.append(episode_reward)

        return {
            "mean_reward": float(np.mean(rewards)),
            "std_reward": float(np.std(rewards)),
            "goals_scored": goals_scored,
            "goals_conceded": goals_conceded,
            "episodes": n_episodes,
        }
