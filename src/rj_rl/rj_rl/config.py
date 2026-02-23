"""Configuration management for the RL system.

Provides RL-specific configuration values. Physical constants (field
dimensions, robot/ball geometry) are not duplicated here — they come
from the shared ``constants`` module which mirrors
``rj_constants/constants.hpp``.
"""
from dataclasses import dataclass, field as dataclass_field
from typing import Dict, Any, List


@dataclass
class RewardConfig:
    """Reward function weights."""

    goal_scored: float = 10.0
    goal_conceded: float = -10.0
    ball_possession: float = 0.1
    ball_progress: float = 0.05
    time_penalty: float = -0.001


@dataclass
class TrainingConfig:
    """Training hyperparameters."""

    total_timesteps: int = 100000
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_epsilon: float = 0.2
    entropy_coeff: float = 0.01
    value_coeff: float = 0.5
    max_grad_norm: float = 0.5
    batch_size: int = 64
    n_epochs: int = 4
    rollout_steps: int = 2048
    hidden_sizes: List[int] = dataclass_field(default_factory=lambda: [64, 64])
    log_interval: int = 10
    save_interval: int = 50
    seed: int = 42


@dataclass
class EnvConfig:
    """Environment configuration."""

    num_blue_robots: int = 6
    num_yellow_robots: int = 6
    max_episode_steps: int = 3000
    controlled_team: str = "blue"


@dataclass
class RLConfig:
    """Top-level RL configuration container.

    Physical constants (field dimensions, robot/ball radius, etc.) are
    provided by the ``constants`` module and are NOT duplicated here.
    """

    reward: RewardConfig = dataclass_field(default_factory=RewardConfig)
    training: TrainingConfig = dataclass_field(default_factory=TrainingConfig)
    env: EnvConfig = dataclass_field(default_factory=EnvConfig)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "RLConfig":
        """Create configuration from a nested dictionary.

        Unknown keys are silently ignored, making it easy to override
        only the parameters you care about.
        """
        cfg = cls()
        section_map = {
            "reward": (cfg.reward, RewardConfig),
            "training": (cfg.training, TrainingConfig),
            "env": (cfg.env, EnvConfig),
        }
        for section_name, (section_obj, _section_cls) in section_map.items():
            if section_name in d:
                for key, value in d[section_name].items():
                    if hasattr(section_obj, key):
                        setattr(section_obj, key, value)
        return cfg
