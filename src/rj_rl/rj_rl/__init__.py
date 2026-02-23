"""rj_rl: Reinforcement learning system for RoboCup SSL strategy.

This package provides an RL framework for learning RoboCup strategy that
integrates with the existing grSim simulator. Physical constants (field
dimensions, robot/ball geometry) are sourced from a shared ``constants``
module that mirrors ``rj_constants/constants.hpp``, and the environment
communicates with grSim via the same UDP protobuf protocol as the C++
``sim_radio`` node.

Modules:
    constants:  Shared physical constants (mirrors rj_constants/constants.hpp)
    env:        Gym-compatible environment backed by grSim
    state:      State encoding/decoding for observation space
    action:     Action space definitions
    reward:     Configurable reward functions
    network:    Neural network implementation (numpy-based)
    agent:      PPO reinforcement learning agent
    trainer:    Training loop with logging and checkpointing
    config:     RL-specific configuration management
    sim_client: gRSim UDP protobuf communication layer
"""

__version__ = "0.1.0"
