"""Gym-compatible RoboCup SSL environment with built-in 2D physics.

Provides a self-contained simulation of a simplified RoboCup Small-Sized
League game. The environment follows the standard Gym interface (reset,
step) and does not require any external simulators to run, making it
suitable for rapid RL training and prototyping.

Physics model:
    - Robots: velocity-based movement with acceleration limits
    - Ball: 2D motion with friction, elastic collisions with walls/robots
    - Goals: detected when ball fully crosses the goal line
"""
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .action import ActionType, action_to_target
from .config import RLConfig
from .reward import RewardComputer
from .state import StateEncoder


@dataclass
class Robot:
    """State of a single robot."""

    pos: np.ndarray  # (x, y) meters
    vel: np.ndarray  # (vx, vy) m/s
    heading: float  # radians

    def copy(self) -> "Robot":
        return Robot(self.pos.copy(), self.vel.copy(), self.heading)


@dataclass
class Ball:
    """State of the ball."""

    pos: np.ndarray  # (x, y) meters
    vel: np.ndarray  # (vx, vy) m/s

    def copy(self) -> "Ball":
        return Ball(self.pos.copy(), self.vel.copy())


class RoboCupEnv:
    """Gym-compatible RoboCup SSL environment.

    Controls a single robot from the blue team. The remaining blue robots
    and all yellow robots follow simple heuristic behaviors to provide a
    realistic training environment.

    Usage:
        >>> config = RLConfig()
        >>> env = RoboCupEnv(config)
        >>> obs = env.reset()
        >>> obs, reward, done, info = env.step(0)

    Args:
        config: Environment configuration.
        controlled_robot_id: Index of the robot controlled by the RL agent.
    """

    def __init__(
        self,
        config: Optional[RLConfig] = None,
        controlled_robot_id: int = 0,
    ):
        if config is None:
            config = RLConfig()

        self.config = config
        self.controlled_id = controlled_robot_id

        # Field geometry
        self._half_length = config.field.length / 2.0
        self._half_width = config.field.width / 2.0
        self._goal_half_width = config.field.goal_width / 2.0

        # Goal positions (blue defends left, yellow defends right)
        self._blue_goal_x = -self._half_length
        self._yellow_goal_x = self._half_length

        # State encoder
        self.state_encoder = StateEncoder(
            config.field,
            num_teammates=config.env.num_blue_robots - 1,
            num_opponents=config.env.num_yellow_robots,
        )

        # Reward computer
        self.reward_computer = RewardComputer(
            config.reward,
            opp_goal_x=self._yellow_goal_x,
            field_length=config.field.length,
        )

        # Spaces (Gym-compatible descriptors)
        self.observation_size = self.state_encoder.observation_size
        self.action_size = ActionType.size()

        # State
        self.blue_robots = []
        self.yellow_robots = []
        self.ball = Ball(np.zeros(2), np.zeros(2))
        self.step_count = 0
        self._prev_ball_pos = np.zeros(2)
        self.rng = np.random.default_rng(config.training.seed)

    def reset(self, seed: Optional[int] = None) -> np.ndarray:
        """Reset environment to a random initial state.

        Args:
            seed: Optional random seed override.

        Returns:
            Initial observation vector.
        """
        if seed is not None:
            self.rng = np.random.default_rng(seed)

        self.step_count = 0

        # Initialize ball at center with small random offset
        self.ball = Ball(
            pos=self.rng.uniform(-0.5, 0.5, size=2).astype(np.float32),
            vel=np.zeros(2, dtype=np.float32),
        )
        self._prev_ball_pos = self.ball.pos.copy()

        # Initialize blue robots in their half
        self.blue_robots = []
        for i in range(self.config.env.num_blue_robots):
            x = self.rng.uniform(-self._half_length + 0.5, -0.5)
            y = self.rng.uniform(-self._half_width + 0.5, self._half_width - 0.5)
            heading = float(self.rng.uniform(0, 2 * np.pi))
            self.blue_robots.append(
                Robot(
                    pos=np.array([x, y], dtype=np.float32),
                    vel=np.zeros(2, dtype=np.float32),
                    heading=heading,
                )
            )

        # Initialize yellow robots in their half
        self.yellow_robots = []
        for i in range(self.config.env.num_yellow_robots):
            x = self.rng.uniform(0.5, self._half_length - 0.5)
            y = self.rng.uniform(-self._half_width + 0.5, self._half_width - 0.5)
            heading = float(self.rng.uniform(0, 2 * np.pi))
            self.yellow_robots.append(
                Robot(
                    pos=np.array([x, y], dtype=np.float32),
                    vel=np.zeros(2, dtype=np.float32),
                    heading=heading,
                )
            )

        return self._get_obs()

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """Execute one environment step.

        Args:
            action: Discrete action index (see ActionType).

        Returns:
            Tuple of (observation, reward, done, info).
        """
        self._prev_ball_pos = self.ball.pos.copy()
        self.step_count += 1

        # Apply action to controlled robot
        controlled = self.blue_robots[self.controlled_id]
        teammate_positions = self._get_teammate_positions()
        own_goal = np.array([self._blue_goal_x, 0.0])
        opp_goal = np.array([self._yellow_goal_x, 0.0])

        target, should_kick = action_to_target(
            action,
            controlled.pos,
            self.ball.pos,
            teammate_positions,
            own_goal,
            opp_goal,
            self.config.field.length,
            self.config.field.width,
        )

        self._move_robot_toward(controlled, target)

        # Handle kicking
        if should_kick:
            self._try_kick(controlled, opp_goal)

        # Move heuristic robots
        self._update_heuristic_blue()
        self._update_heuristic_yellow()

        # Update ball physics
        self._update_ball()

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
        blue_positions = np.array([r.pos for r in self.blue_robots])
        reward = self.reward_computer.compute(
            self.ball.pos,
            self._prev_ball_pos,
            blue_positions,
            goal_scored,
            goal_conceded,
        )

        obs = self._get_obs()
        return obs, reward, done, info

    # --- Internal helpers ---

    def _get_obs(self) -> np.ndarray:
        """Build observation for the controlled robot."""
        controlled = self.blue_robots[self.controlled_id]
        return self.state_encoder.encode(
            robot_pos=controlled.pos,
            robot_vel=controlled.vel,
            robot_heading=controlled.heading,
            ball_pos=self.ball.pos,
            ball_vel=self.ball.vel,
            teammate_positions=self._get_teammate_positions(),
            opponent_positions=np.array(
                [r.pos for r in self.yellow_robots]
            ),
            own_goal_x=self._blue_goal_x,
            opp_goal_x=self._yellow_goal_x,
        )

    def _get_teammate_positions(self) -> np.ndarray:
        """Get positions of blue robots excluding the controlled one."""
        positions = []
        for i, r in enumerate(self.blue_robots):
            if i != self.controlled_id:
                positions.append(r.pos)
        return np.array(positions) if positions else np.empty((0, 2))

    def _move_robot_toward(self, robot: Robot, target: np.ndarray) -> None:
        """Move a robot toward a target position with acceleration limits."""
        dt = self.config.physics.dt
        max_speed = self.config.physics.robot_max_speed
        max_accel = self.config.physics.robot_max_acceleration

        direction = target - robot.pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            # Close enough: decelerate
            robot.vel *= 0.9
        else:
            desired_vel = direction / dist * min(dist / dt, max_speed)
            accel = (desired_vel - robot.vel) / dt
            accel_mag = np.linalg.norm(accel)
            if accel_mag > max_accel:
                accel = accel / accel_mag * max_accel
            robot.vel = robot.vel + accel * dt

        # Clamp speed
        speed = np.linalg.norm(robot.vel)
        if speed > max_speed:
            robot.vel = robot.vel / speed * max_speed

        robot.pos = robot.pos + robot.vel * dt

        # Update heading to face movement direction
        if speed > 0.1:
            robot.heading = float(np.arctan2(robot.vel[1], robot.vel[0]))

        # Clamp to field bounds
        robot.pos[0] = np.clip(
            robot.pos[0],
            -self._half_length + self.config.physics.robot_radius,
            self._half_length - self.config.physics.robot_radius,
        )
        robot.pos[1] = np.clip(
            robot.pos[1],
            -self._half_width + self.config.physics.robot_radius,
            self._half_width - self.config.physics.robot_radius,
        )

    def _try_kick(self, robot: Robot, target: np.ndarray) -> None:
        """Kick the ball toward a target if the robot is close enough."""
        dist_to_ball = np.linalg.norm(robot.pos - self.ball.pos)
        kick_range = (
            self.config.physics.robot_radius + self.config.physics.ball_radius + 0.02
        )
        if dist_to_ball < kick_range:
            direction = target - self.ball.pos
            dist = np.linalg.norm(direction)
            if dist > 1e-6:
                direction = direction / dist
            self.ball.vel = direction * self.config.physics.kick_speed

    def _update_ball(self) -> None:
        """Update ball position with friction and wall bouncing."""
        dt = self.config.physics.dt
        friction = self.config.physics.ball_friction

        # Apply friction
        speed = np.linalg.norm(self.ball.vel)
        if speed > 0:
            decel = min(friction * dt, speed)
            self.ball.vel = self.ball.vel * (1.0 - decel / speed)

        self.ball.pos = self.ball.pos + self.ball.vel * dt

        # Clamp speed
        speed = np.linalg.norm(self.ball.vel)
        if speed > self.config.physics.ball_max_speed:
            self.ball.vel = (
                self.ball.vel / speed * self.config.physics.ball_max_speed
            )

        # Wall bouncing (y-axis)
        if abs(self.ball.pos[1]) > self._half_width:
            self.ball.pos[1] = np.clip(
                self.ball.pos[1], -self._half_width, self._half_width
            )
            self.ball.vel[1] *= -0.7

        # Wall bouncing (x-axis, outside goal)
        if abs(self.ball.pos[0]) > self._half_length:
            if abs(self.ball.pos[1]) > self._goal_half_width:
                self.ball.pos[0] = np.clip(
                    self.ball.pos[0], -self._half_length, self._half_length
                )
                self.ball.vel[0] *= -0.7

        # Robot-ball collisions
        for robots in [self.blue_robots, self.yellow_robots]:
            for robot in robots:
                collision_dist = (
                    self.config.physics.robot_radius
                    + self.config.physics.ball_radius
                )
                diff = self.ball.pos - robot.pos
                dist = np.linalg.norm(diff)
                if dist < collision_dist and dist > 1e-6:
                    # Push ball out and reflect velocity
                    normal = diff / dist
                    self.ball.pos = robot.pos + normal * collision_dist
                    relative_vel = self.ball.vel - robot.vel
                    self.ball.vel = (
                        self.ball.vel
                        - 1.5 * np.dot(relative_vel, normal) * normal
                        + robot.vel * 0.3
                    )

    def _is_goal_at(self, goal_x: float) -> bool:
        """Check if the ball has crossed the given goal line."""
        in_goal_y = abs(self.ball.pos[1]) < self._goal_half_width
        if goal_x > 0:
            return bool(self.ball.pos[0] > goal_x and in_goal_y)
        else:
            return bool(self.ball.pos[0] < goal_x and in_goal_y)

    def _update_heuristic_blue(self) -> None:
        """Move non-controlled blue robots with simple heuristics."""
        for i, robot in enumerate(self.blue_robots):
            if i == self.controlled_id:
                continue

            # Simple behavior: spread out and loosely track ball
            base_y = (
                (i - self.config.env.num_blue_robots / 2)
                * self.config.field.width
                / (self.config.env.num_blue_robots + 1)
            )
            target_x = np.clip(
                self.ball.pos[0] - 1.5,
                -self._half_length + 1.0,
                0.0,
            )
            target_y = 0.7 * base_y + 0.3 * self.ball.pos[1]
            self._move_robot_toward(
                robot, np.array([target_x, target_y], dtype=np.float32)
            )

    def _update_heuristic_yellow(self) -> None:
        """Move yellow robots with simple defensive heuristics."""
        for i, robot in enumerate(self.yellow_robots):
            if i == 0:
                # Goalie: stay near goal, track ball y
                target = np.array(
                    [
                        self._yellow_goal_x - 0.3,
                        np.clip(
                            self.ball.pos[1],
                            -self._goal_half_width + 0.05,
                            self._goal_half_width - 0.05,
                        ),
                    ],
                    dtype=np.float32,
                )
            else:
                # Defenders: spread in front of goal
                base_y = (
                    (i - self.config.env.num_yellow_robots / 2)
                    * self.config.field.width
                    / (self.config.env.num_yellow_robots + 1)
                )
                target_x = np.clip(
                    self.ball.pos[0] + 1.0,
                    0.0,
                    self._yellow_goal_x - 1.0,
                )
                target_y = 0.6 * base_y + 0.4 * self.ball.pos[1]
                target = np.array([target_x, target_y], dtype=np.float32)

            self._move_robot_toward(robot, target)
