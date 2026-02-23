"""rj_rl: Reinforcement learning system for RoboCup SSL strategy.

This package provides a self-contained RL framework for learning robocup
strategy using a built-in 2D physics simulation. The trained policies can
be integrated with the existing ROS2-based strategy system.

Modules:
    env:      Gym-compatible environment with 2D physics simulation
    state:    State encoding/decoding for observation space
    action:   Action space definitions
    reward:   Configurable reward functions
    network:  Neural network implementation (numpy-based)
    agent:    PPO reinforcement learning agent
    trainer:  Training loop with logging and checkpointing
    config:   Configuration management
"""

__version__ = "0.1.0"
