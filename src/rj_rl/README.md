# rj_rl — Reinforcement Learning for RoboCup SSL Strategy

A self-contained reinforcement learning framework for learning RoboCup
Small-Sized League (SSL) strategy. Uses a built-in 2D physics simulation
and a PPO (Proximal Policy Optimization) agent implemented entirely with
numpy, so no external deep learning frameworks are required.

## Quick Start

```bash
# From the rj_rl directory
cd src/rj_rl

# Install in development mode
pip install -e .

# Run training (short run for testing)
python -m scripts.train --timesteps 5000 --seed 42

# Run tests
python -m pytest test/ -v
```

## Architecture

```
rj_rl/
├── env.py        Gym-compatible 2D environment with SSL-like physics
├── state.py      State encoding: world state → normalized observation vector
├── action.py     Discrete action space (move, shoot, pass, defend, …)
├── reward.py     Composable reward functions (goals, possession, progress)
├── network.py    Numpy-based MLP (actor-critic policy network)
├── agent.py      PPO agent with GAE advantage estimation
├── trainer.py    Training loop with logging and checkpointing
└── config.py     Dataclass-based configuration (field, physics, rewards, training)
```

## How It Works

### Environment (`env.py`)
The environment simulates a simplified RoboCup SSL game:
- 2D field with configurable dimensions (default 9m × 6m)
- Up to 6 robots per team with acceleration-based movement
- Ball physics with friction, wall bouncing, and robot collisions
- Goal detection when the ball crosses the goal line
- Heuristic opponents and teammates for realistic dynamics

The RL agent controls **one robot** and receives observations about the
full game state (robot positions/velocities, ball state, goal positions).

### Actions (`action.py`)
Seven discrete actions:
| Action              | Description                             |
|---------------------|-----------------------------------------|
| MOVE_TO_BALL        | Navigate toward the ball                |
| SHOOT_ON_GOAL       | Move to ball and kick toward opponent goal |
| PASS_TO_NEAREST_TEAMMATE | Move to ball and kick to a teammate |
| DEFEND_GOAL         | Position between ball and own goal      |
| POSITION_OFFENSE    | Move to an offensive position           |
| POSITION_DEFENSE    | Move to a defensive position            |
| IDLE                | Stay in current position                |

### Rewards (`reward.py`)
The reward function is a weighted sum of:
- **Goal scored**: large positive reward (+10)
- **Goal conceded**: large negative reward (−10)
- **Ball possession**: reward for proximity to the ball
- **Ball progress**: reward for moving the ball toward opponent goal
- **Time penalty**: small negative per step (encourages efficiency)

All weights are configurable via `RewardConfig`.

### Agent (`agent.py`)
PPO with:
- Actor-critic architecture (separate MLPs for policy and value)
- Generalized Advantage Estimation (GAE)
- Clipped surrogate objective for stable updates
- Adam optimizer with numerical gradient estimation

## Configuration

All parameters are exposed through dataclasses in `config.py`:

```python
from rj_rl.config import RLConfig

config = RLConfig()
config.training.learning_rate = 1e-3
config.training.total_timesteps = 200000
config.reward.goal_scored = 20.0
config.env.num_blue_robots = 3
```

## Integration with Existing Strategy

The trained policy can be integrated with the existing ROS2 strategy
system by:

1. **Loading a trained model** in a new ROS2 node that subscribes to
   `world_state` and `play_state` topics.
2. **Converting ROS2 messages** to the observation format using
   `StateEncoder`.
3. **Querying the policy** with `agent.select_action(obs)` to get
   the action.
4. **Publishing the resulting action** as a `RobotIntent` message.

This can replace or augment existing Position implementations in
`rj_strategy` without modifying the core planning/control pipeline.

## Extending

### Adding new actions
1. Add an entry to `ActionType` in `action.py`
2. Add the corresponding target computation in `action_to_target()`
3. Re-train the agent

### Changing the reward function
1. Modify weights in `RewardConfig` or add new reward components
   to `RewardComputer.compute()`
2. Re-train the agent

### Using a different RL algorithm
1. Implement a new agent class following the same interface as
   `PPOAgent` (methods: `select_action`, `update`, `save`, `load`)
2. Pass it to `Trainer` or use it directly

### Switching to PyTorch / TensorFlow
Replace `NumpyMLP` in `network.py` with a framework-native network.
The `PolicyNetwork` interface (`get_action_and_value`, `get_value`,
`save`, `load`) remains the same.
