# rj_rl — Reinforcement Learning for RoboCup SSL Strategy

A reinforcement learning framework for learning RoboCup Small-Sized League
(SSL) strategy. Integrates with the existing **grSim** simulator (the same
one used by the rest of the stack) and reuses physical constants from
`rj_constants/constants.hpp` — no duplicated physics or geometry.

## Quick Start

```bash
# From the rj_rl directory
cd src/rj_rl

# Install in development mode
pip install -e .

# Run training (short run, uses fallback physics when grSim is not running)
python -m scripts.train --timesteps 5000 --seed 42

# Run training with grSim (start grSim first)
python -m scripts.train --timesteps 50000 --use-sim

# Run tests
python -m pytest test/ -v
```

## Architecture

```
rj_rl/
├── constants.py   Shared physical constants (mirrors rj_constants/constants.hpp)
├── env.py         Gym-compatible environment backed by grSim
├── sim_client.py  gRSim UDP protobuf communication (same protocol as sim_radio.cpp)
├── state.py       State encoding: world state → normalized observation vector
├── action.py      Discrete action space (move, shoot, pass, defend, …)
├── reward.py      Composable reward functions (goals, possession, progress)
├── network.py     Numpy-based MLP (actor-critic policy network)
├── agent.py       PPO agent with GAE advantage estimation
├── trainer.py     Training loop with logging and checkpointing
├── config.py      RL-specific configuration (reward weights, training params)
└── proto_gen/     Generated Python protobuf bindings for SSL simulator protocol
```

## Integration with the Existing Simulator

The RL environment communicates with **grSim** using the same UDP protobuf
protocol as the C++ `sim_radio` node (`src/rj_radio/src/sim_radio.cpp`):

| Protocol Message     | Direction | Purpose                        |
|---------------------|-----------|--------------------------------|
| `RobotControl`      | → grSim   | Robot velocity/kick commands   |
| `SSL_WrapperPacket` | ← grSim   | Vision data (robot/ball state) |
| `SimulatorCommand`  | → grSim   | Teleport ball/robots (resets)  |

Network ports match the existing configuration in
`rj_common/include/rj_common/network.hpp`:
- Vision: 10020, Commands: 10301/10302, Control: 10300

## Shared Constants (No Duplication)

Physical constants are defined **once** in `constants.py`, mirroring the
values from the C++ codebase:

| Constant          | Source                          | Value     |
|-------------------|---------------------------------|-----------|
| `ROBOT_RADIUS`    | `rj_constants/constants.hpp`    | 0.090 m   |
| `BALL_RADIUS`     | `rj_constants/constants.hpp`    | 0.0215 m  |
| `BALL_DECEL`      | `rj_constants/constants.hpp`    | −0.4 m/s² |
| `FIELD_LENGTH`    | SSL Division B standard         | 9.0 m     |
| `FIELD_WIDTH`     | SSL Division B standard         | 6.0 m     |
| `GOAL_WIDTH`      | SSL Division B standard         | 1.0 m     |
| `ROBOTS_PER_TEAM` | `rj_constants/constants.hpp`    | 6         |
| `SIM_*_PORT`      | `rj_common/network.hpp`         | 10020 etc |

## How It Works

### Environment (`env.py`)
The environment provides a Gym interface (`reset`, `step`) backed by grSim:
- **With grSim**: Sends robot commands and reads vision data via UDP
- **Fallback mode**: Lightweight internal physics using shared constants
  (for testing/CI when grSim is not running)

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

Only RL-specific parameters are configured here. Physical constants come
from `constants.py` (mirroring the C++ codebase):

```python
from rj_rl.config import RLConfig

config = RLConfig()
config.training.learning_rate = 1e-3
config.training.total_timesteps = 200000
config.reward.goal_scored = 20.0
config.env.num_blue_robots = 3
```

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

### Regenerating protobuf bindings
If the `.proto` files in `src/rj_protos/protos/` change:
```bash
python -m grpc_tools.protoc \
  --proto_path=src/rj_protos/protos \
  --python_out=src/rj_rl/rj_rl/proto_gen \
  ssl_simulation_robot_control.proto ssl_simulation_control.proto \
  ssl_gc_common.proto ssl_vision_wrapper.proto ssl_vision_detection.proto \
  ssl_vision_geometry.proto ssl_simulation_config.proto ssl_simulation_error.proto
```
