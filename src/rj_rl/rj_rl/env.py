"""Gym-compatible RoboCup SSL environment backed by the grSim simulator.

Instead of implementing its own physics engine, this environment
communicates with the existing grSim (or ER-Force) simulator via UDP
protobuf — the same protocol used by the C++ ``sim_radio`` node. This
ensures that RL training uses the same physics model as the rest of the
stack.

Field dimensions and robot/ball constants come from the shared
``constants`` module (mirroring ``rj_constants/constants.hpp``).

When no simulator is available (e.g. in unit tests), the environment
falls back to a lightweight built-in simulation that reuses the same
shared constants.
"""
from typing import Any, Dict, Optional, Tuple

import numpy as np

from . import constants
from .action import ActionType, action_to_target
from .config import RLConfig
from .reward import RewardComputer
from .state import StateEncoder


class RoboCupEnv:
    """Gym-compatible RoboCup SSL environment.

    Controls a single robot from the blue team. The remaining blue robots
    and all yellow robots are moved by the simulator (grSim) or by
    simple heuristics in fallback mode.

    The environment tries to connect to grSim on construction. If the
    simulator is not available it uses a lightweight built-in fallback
    that reuses the shared constants from ``rj_constants``.

    Usage:
        >>> config = RLConfig()
        >>> env = RoboCupEnv(config)
        >>> obs = env.reset()
        >>> obs, reward, done, info = env.step(0)

    Args:
        config: RL configuration (reward weights, training params, etc.).
        controlled_robot_id: Index of the robot controlled by the RL agent.
        use_sim: If True attempt grSim connection; if False use fallback.
    """

    def __init__(
        self,
        config: Optional[RLConfig] = None,
        controlled_robot_id: int = 0,
        use_sim: bool = False,
    ):
        if config is None:
            config = RLConfig()

        self.config = config
        self.controlled_id = controlled_robot_id
        self.use_sim = use_sim

        # Field geometry from shared constants
        self._half_length = constants.HALF_FIELD_LENGTH
        self._half_width = constants.HALF_FIELD_WIDTH
        self._goal_half_width = constants.HALF_GOAL_WIDTH
        self._blue_goal_x = constants.BLUE_GOAL_X
        self._yellow_goal_x = constants.YELLOW_GOAL_X

        # State encoder (uses shared constants internally)
        self.state_encoder = StateEncoder(
            num_teammates=config.env.num_blue_robots - 1,
            num_opponents=config.env.num_yellow_robots,
        )

        # Reward computer
        self.reward_computer = RewardComputer(
            config.reward,
            opp_goal_x=self._yellow_goal_x,
            field_length=constants.FIELD_LENGTH,
        )

        # Spaces (Gym-compatible descriptors)
        self.observation_size = self.state_encoder.observation_size
        self.action_size = ActionType.size()

        # Simulator client (lazy-initialised)
        self._sim_client = None

        # Fallback internal state (used when grSim is not available)
        self.blue_robots: list = []
        self.yellow_robots: list = []
        self.ball_pos = np.zeros(2, dtype=np.float32)
        self.ball_vel = np.zeros(2, dtype=np.float32)
        self.step_count = 0
        self._prev_ball_pos = np.zeros(2, dtype=np.float32)
        self.rng = np.random.default_rng(config.training.seed)

        # Try to connect to grSim if requested
        if self.use_sim:
            self._init_sim_client()

    def _init_sim_client(self) -> None:
        """Attempt to connect to the grSim simulator."""
        try:
            from .sim_client import GrSimClient

            self._sim_client = GrSimClient(blue_team=True)
            self._sim_client.connect()
        except Exception:
            self._sim_client = None

    # ------------------------------------------------------------------
    # Gym interface
    # ------------------------------------------------------------------

    def reset(self, seed: Optional[int] = None) -> np.ndarray:
        """Reset environment to a random initial state.

        If connected to grSim, teleports robots and ball to random
        starting positions using the simulator's placement API.
        Otherwise falls back to the internal state.

        Args:
            seed: Optional random seed override.

        Returns:
            Initial observation vector.
        """
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self.step_count = 0

        # Generate random starting positions
        ball_pos = self.rng.uniform(-0.5, 0.5, size=2).astype(np.float32)
        blue_positions = []
        for _ in range(self.config.env.num_blue_robots):
            x = self.rng.uniform(-self._half_length + 0.5, -0.5)
            y = self.rng.uniform(
                -self._half_width + 0.5, self._half_width - 0.5
            )
            heading = float(self.rng.uniform(0, 2 * np.pi))
            blue_positions.append((x, y, heading))

        yellow_positions = []
        for _ in range(self.config.env.num_yellow_robots):
            x = self.rng.uniform(0.5, self._half_length - 0.5)
            y = self.rng.uniform(
                -self._half_width + 0.5, self._half_width - 0.5
            )
            heading = float(self.rng.uniform(0, 2 * np.pi))
            yellow_positions.append((x, y, heading))

        if self._sim_client is not None and self._sim_client.connected:
            self._reset_sim(ball_pos, blue_positions, yellow_positions)
        else:
            self._reset_fallback(ball_pos, blue_positions, yellow_positions)

        return self._get_obs()

    def step(
        self, action: int
    ) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Execute one environment step.

        Args:
            action: Discrete action index (see ActionType).

        Returns:
            Tuple of (observation, reward, done, info).
        """
        self._prev_ball_pos = self.ball_pos.copy()
        self.step_count += 1

        if self._sim_client is not None and self._sim_client.connected:
            self._step_sim(action)
        else:
            self._step_fallback(action)

        # Check for goals
        goal_scored = False
        goal_conceded = False
        done = False
        info: Dict[str, Any] = {}

        if self._is_goal_at(self._yellow_goal_x):
            goal_scored = True
            done = True
            info["result"] = "goal_scored"
        elif self._is_goal_at(self._blue_goal_x):
            goal_conceded = True
            done = True
            info["result"] = "goal_conceded"

        if self.step_count >= self.config.env.max_episode_steps:
            done = True
            info.setdefault("result", "timeout")

        # Compute reward
        blue_positions = np.array(
            [r["pos"] for r in self.blue_robots]
        )
        reward = self.reward_computer.compute(
            self.ball_pos,
            self._prev_ball_pos,
            blue_positions,
            goal_scored,
            goal_conceded,
        )

        obs = self._get_obs()
        return obs, reward, done, info

    # ------------------------------------------------------------------
    # Observation building
    # ------------------------------------------------------------------

    def _get_obs(self) -> np.ndarray:
        """Build observation for the controlled robot."""
        controlled = self.blue_robots[self.controlled_id]
        teammate_positions = self._get_teammate_positions()
        opponent_positions = np.array(
            [r["pos"] for r in self.yellow_robots]
        )
        return self.state_encoder.encode(
            robot_pos=controlled["pos"],
            robot_vel=controlled["vel"],
            robot_heading=controlled["heading"],
            ball_pos=self.ball_pos,
            ball_vel=self.ball_vel,
            teammate_positions=teammate_positions,
            opponent_positions=opponent_positions,
            own_goal_x=self._blue_goal_x,
            opp_goal_x=self._yellow_goal_x,
        )

    def _get_teammate_positions(self) -> np.ndarray:
        """Get positions of blue robots excluding the controlled one."""
        positions = []
        for i, r in enumerate(self.blue_robots):
            if i != self.controlled_id:
                positions.append(r["pos"])
        return np.array(positions) if positions else np.empty((0, 2))

    def _is_goal_at(self, goal_x: float) -> bool:
        """Check if the ball has crossed the given goal line."""
        in_goal_y = abs(self.ball_pos[1]) < self._goal_half_width
        if goal_x > 0:
            return bool(self.ball_pos[0] > goal_x and in_goal_y)
        else:
            return bool(self.ball_pos[0] < goal_x and in_goal_y)

    # ------------------------------------------------------------------
    # grSim-backed simulation
    # ------------------------------------------------------------------

    def _reset_sim(self, ball_pos, blue_positions, yellow_positions):
        """Reset via grSim teleport commands."""
        self._sim_client.teleport_ball(float(ball_pos[0]), float(ball_pos[1]))
        for i, (x, y, h) in enumerate(blue_positions):
            self._sim_client.teleport_robot(i, True, x, y, h)
        for i, (x, y, h) in enumerate(yellow_positions):
            self._sim_client.teleport_robot(i, False, x, y, h)

        # Wait for vision update
        import time
        time.sleep(0.1)

        world = self._sim_client.receive_world_state()
        self._sync_from_sim(world, ball_pos, blue_positions, yellow_positions)

    def _step_sim(self, action: int):
        """Execute one step via grSim."""
        controlled = self.blue_robots[self.controlled_id]
        teammate_positions = self._get_teammate_positions()
        own_goal = np.array([self._blue_goal_x, 0.0])
        opp_goal = np.array([self._yellow_goal_x, 0.0])

        target, should_kick = action_to_target(
            action,
            controlled["pos"],
            self.ball_pos,
            teammate_positions,
            own_goal,
            opp_goal,
            constants.FIELD_LENGTH,
            constants.FIELD_WIDTH,
        )

        # Convert target to velocity command
        direction = target - controlled["pos"]
        dist = np.linalg.norm(direction)
        max_speed = constants.SIM_ROBOT_MAX_SPEED
        if dist > 0.01:
            speed = min(dist * 3.0, max_speed)
            vx = direction[0] / dist * speed
            vy = direction[1] / dist * speed
        else:
            vx, vy = 0.0, 0.0

        kick_speed = constants.SIM_ROBOT_MAX_KICK_SPEED if should_kick else 0.0

        self._sim_client.send_robot_commands([
            (self.controlled_id, vx, vy, 0.0, kick_speed, 0.0)
        ])

        # Wait one sim step and read state
        import time
        time.sleep(constants.SIM_DT)
        world = self._sim_client.receive_world_state()
        if world is not None:
            self._sync_from_sim(world)

    def _sync_from_sim(self, world, ball_pos=None, blue_pos=None, yellow_pos=None):
        """Synchronize internal state from grSim vision data."""
        if world is None:
            # Use provided initial positions if no vision data yet
            if ball_pos is not None:
                self.ball_pos = ball_pos.copy()
                self.ball_vel = np.zeros(2, dtype=np.float32)
            if blue_pos is not None:
                self.blue_robots = []
                for x, y, h in blue_pos:
                    self.blue_robots.append({
                        "pos": np.array([x, y], dtype=np.float32),
                        "vel": np.zeros(2, dtype=np.float32),
                        "heading": h,
                    })
            if yellow_pos is not None:
                self.yellow_robots = []
                for x, y, h in yellow_pos:
                    self.yellow_robots.append({
                        "pos": np.array([x, y], dtype=np.float32),
                        "vel": np.zeros(2, dtype=np.float32),
                        "heading": h,
                    })
            return

        self.ball_pos = world.ball.pos.copy()
        self.ball_vel = self._sim_client.get_ball_velocity()

        self.blue_robots = []
        for i in range(self.config.env.num_blue_robots):
            if i in world.blue_robots:
                r = world.blue_robots[i]
                self.blue_robots.append({
                    "pos": r.pos.copy(),
                    "vel": self._sim_client.get_robot_velocity(i),
                    "heading": r.orientation,
                })
            else:
                self.blue_robots.append({
                    "pos": np.zeros(2, dtype=np.float32),
                    "vel": np.zeros(2, dtype=np.float32),
                    "heading": 0.0,
                })

        self.yellow_robots = []
        for i in range(self.config.env.num_yellow_robots):
            if i in world.yellow_robots:
                r = world.yellow_robots[i]
                self.yellow_robots.append({
                    "pos": r.pos.copy(),
                    "vel": np.zeros(2, dtype=np.float32),
                    "heading": r.orientation,
                })
            else:
                self.yellow_robots.append({
                    "pos": np.zeros(2, dtype=np.float32),
                    "vel": np.zeros(2, dtype=np.float32),
                    "heading": 0.0,
                })

    # ------------------------------------------------------------------
    # Built-in fallback (when grSim is not available)
    # Uses shared constants — no duplicated physics parameters.
    # ------------------------------------------------------------------

    def _reset_fallback(self, ball_pos, blue_positions, yellow_positions):
        """Reset using internal state (fallback mode)."""
        self.ball_pos = ball_pos.copy()
        self.ball_vel = np.zeros(2, dtype=np.float32)
        self._prev_ball_pos = ball_pos.copy()

        self.blue_robots = []
        for x, y, h in blue_positions:
            self.blue_robots.append({
                "pos": np.array([x, y], dtype=np.float32),
                "vel": np.zeros(2, dtype=np.float32),
                "heading": h,
            })

        self.yellow_robots = []
        for x, y, h in yellow_positions:
            self.yellow_robots.append({
                "pos": np.array([x, y], dtype=np.float32),
                "vel": np.zeros(2, dtype=np.float32),
                "heading": h,
            })

    def _step_fallback(self, action: int):
        """Execute one step using internal physics (fallback mode).

        Uses shared constants from ``rj_constants`` for all physical
        parameters (robot radius, ball radius, friction, etc.).
        """
        controlled = self.blue_robots[self.controlled_id]
        teammate_positions = self._get_teammate_positions()
        own_goal = np.array([self._blue_goal_x, 0.0])
        opp_goal = np.array([self._yellow_goal_x, 0.0])

        target, should_kick = action_to_target(
            action,
            controlled["pos"],
            self.ball_pos,
            teammate_positions,
            own_goal,
            opp_goal,
            constants.FIELD_LENGTH,
            constants.FIELD_WIDTH,
        )

        self._move_robot_toward(controlled, target)

        if should_kick:
            self._try_kick(controlled, opp_goal)

        # Move heuristic robots
        self._update_heuristic_blue()
        self._update_heuristic_yellow()

        # Update ball physics
        self._update_ball()

    def _move_robot_toward(self, robot: dict, target: np.ndarray) -> None:
        """Move a robot toward a target position with acceleration limits.

        Uses shared constants for max speed, acceleration, and robot radius.
        """
        dt = constants.SIM_DT
        max_speed = constants.SIM_ROBOT_MAX_SPEED
        max_accel = constants.SIM_ROBOT_MAX_ACCELERATION

        direction = target - robot["pos"]
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            robot["vel"] *= 0.9
        else:
            desired_vel = direction / dist * min(dist / dt, max_speed)
            accel = (desired_vel - robot["vel"]) / dt
            accel_mag = np.linalg.norm(accel)
            if accel_mag > max_accel:
                accel = accel / accel_mag * max_accel
            robot["vel"] = robot["vel"] + accel * dt

        speed = np.linalg.norm(robot["vel"])
        if speed > max_speed:
            robot["vel"] = robot["vel"] / speed * max_speed

        robot["pos"] = robot["pos"] + robot["vel"] * dt

        if speed > 0.1:
            robot["heading"] = float(
                np.arctan2(robot["vel"][1], robot["vel"][0])
            )

        robot["pos"][0] = np.clip(
            robot["pos"][0],
            -self._half_length + constants.ROBOT_RADIUS,
            self._half_length - constants.ROBOT_RADIUS,
        )
        robot["pos"][1] = np.clip(
            robot["pos"][1],
            -self._half_width + constants.ROBOT_RADIUS,
            self._half_width - constants.ROBOT_RADIUS,
        )

    def _try_kick(self, robot: dict, target: np.ndarray) -> None:
        """Kick the ball toward a target if the robot is close enough.

        Uses shared constants for robot/ball radius and kick speed.
        """
        dist_to_ball = np.linalg.norm(robot["pos"] - self.ball_pos)
        kick_range = constants.ROBOT_RADIUS + constants.BALL_RADIUS + 0.02
        if dist_to_ball < kick_range:
            direction = target - self.ball_pos
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                direction = direction / dist
            self.ball_vel = direction * constants.SIM_ROBOT_MAX_KICK_SPEED

    def _update_ball(self) -> None:
        """Update ball position with friction and wall bouncing.

        Uses shared constants for ball deceleration and max speed.
        """
        dt = constants.SIM_DT
        friction = abs(constants.BALL_DECEL)

        speed = np.linalg.norm(self.ball_vel)
        if speed > 0:
            decel = min(friction * dt, speed)
            self.ball_vel = self.ball_vel * (1.0 - decel / speed)

        self.ball_pos = self.ball_pos + self.ball_vel * dt

        speed = np.linalg.norm(self.ball_vel)
        if speed > constants.SIM_ROBOT_MAX_KICK_SPEED:
            self.ball_vel = (
                self.ball_vel / speed * constants.SIM_ROBOT_MAX_KICK_SPEED
            )

        # Wall bouncing (y-axis)
        if abs(self.ball_pos[1]) > self._half_width:
            self.ball_pos[1] = np.clip(
                self.ball_pos[1], -self._half_width, self._half_width
            )
            self.ball_vel[1] *= -0.7

        # Wall bouncing (x-axis, outside goal)
        if abs(self.ball_pos[0]) > self._half_length:
            if abs(self.ball_pos[1]) > self._goal_half_width:
                self.ball_pos[0] = np.clip(
                    self.ball_pos[0],
                    -self._half_length,
                    self._half_length,
                )
                self.ball_vel[0] *= -0.7

        # Robot-ball collisions
        for robots in [self.blue_robots, self.yellow_robots]:
            for robot in robots:
                collision_dist = constants.ROBOT_RADIUS + constants.BALL_RADIUS
                diff = self.ball_pos - robot["pos"]
                dist = np.linalg.norm(diff)
                if dist < collision_dist and dist > 1e-6:
                    normal = diff / dist
                    self.ball_pos = robot["pos"] + normal * collision_dist
                    relative_vel = self.ball_vel - robot["vel"]
                    self.ball_vel = (
                        self.ball_vel
                        - 1.5 * np.dot(relative_vel, normal) * normal
                        + robot["vel"] * 0.3
                    )

    def _update_heuristic_blue(self) -> None:
        """Move non-controlled blue robots with simple heuristics."""
        for i, robot in enumerate(self.blue_robots):
            if i == self.controlled_id:
                continue
            base_y = (
                (i - self.config.env.num_blue_robots / 2)
                * constants.FIELD_WIDTH
                / (self.config.env.num_blue_robots + 1)
            )
            target_x = np.clip(
                self.ball_pos[0] - 1.5,
                -self._half_length + 1.0,
                0.0,
            )
            target_y = 0.7 * base_y + 0.3 * self.ball_pos[1]
            self._move_robot_toward(
                robot, np.array([target_x, target_y], dtype=np.float32)
            )

    def _update_heuristic_yellow(self) -> None:
        """Move yellow robots with simple defensive heuristics."""
        for i, robot in enumerate(self.yellow_robots):
            if i == 0:
                target = np.array(
                    [
                        self._yellow_goal_x - 0.3,
                        np.clip(
                            self.ball_pos[1],
                            -self._goal_half_width + 0.05,
                            self._goal_half_width - 0.05,
                        ),
                    ],
                    dtype=np.float32,
                )
            else:
                base_y = (
                    (i - self.config.env.num_yellow_robots / 2)
                    * constants.FIELD_WIDTH
                    / (self.config.env.num_yellow_robots + 1)
                )
                target_x = np.clip(
                    self.ball_pos[0] + 1.0,
                    0.0,
                    self._yellow_goal_x - 1.0,
                )
                target_y = 0.6 * base_y + 0.4 * self.ball_pos[1]
                target = np.array([target_x, target_y], dtype=np.float32)
            self._move_robot_toward(robot, target)
