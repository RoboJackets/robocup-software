from enum import Enum, auto

import numpy as np
import stp.rc
import stp.role
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import pivot_kick, receive


class State(Enum):
    INIT = auto()
    CAPTURING = auto()
    INIT_SHOOT = auto()
    SHOOTING = auto()
    KICK_DONE = auto()


# TODO: move params to file
OPPONENT_SPEED = 1.5
KICK_SPEED = 4.5
EFF_BLOCK_WIDTH = 0.7


class StrikerRole(stp.role.Role):
    """Grabs ball and shoots on goal. Should eventually be merged with some hybrid PassOrShoot role."""

    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.receive_skill = None
        self.pivot_kick_skill = None

        self._state = State.INIT

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        intent = None

        if self._state == State.INIT:
            # taken from role/passer.py
            self.receive_skill = receive.Receive(robot=self.robot)
            intent = self.receive_skill.tick(world_state)
            self._state = State.CAPTURING
        elif self._state == State.CAPTURING:
            intent = self.receive_skill.tick(world_state)
            if self.receive_skill.is_done(world_state):
                self._state = State.INIT_SHOOT
        elif self._state == State.INIT_SHOOT:
            # TODO: make these params configurable
            shot_kick_speed = 4.5  # TODO: adjust based on dist from target_point
            best_shot_target_point = self._find_target_point(
                world_state, shot_kick_speed
            )

            self.pivot_kick_skill = pivot_kick.PivotKick(
                robot=self.robot,
                target_point=best_shot_target_point,
                chip=False,
                kick_speed=shot_kick_speed,
            )
            self._state = State.SHOOTING
        elif self._state == State.SHOOTING:
            # taken from role/passer.py
            intent = self.pivot_kick_skill.tick(world_state)

            if self.pivot_kick_skill.is_done(world_state):
                self._state = State.KICK_DONE
                # end FSM

        return intent

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == State.KICK_DONE

    def _blocker_margin(
        self,
        kick_origin: np.array,
        kick_target: np.array,
        kick_speed: float,
        blocker: stp.rc.Robot,
    ):
        if not blocker.visible:
            return np.inf

        kick_vector = kick_target - kick_origin
        kick_dist = np.linalg.norm(kick_vector)
        kick_vector /= kick_dist
        # unused ??
        # kick_perp = np.array([kick_vector[1], -kick_vector[0]])

        blocker_position = blocker.pose[0:2]

        # Calculate blocker intercept
        blocker_intercept_dist_along_kick = np.dot(
            blocker_position - kick_origin, kick_vector
        )
        blocker_intercept_dist_along_kick = np.clip(
            blocker_intercept_dist_along_kick, a_min=0, a_max=kick_dist
        )
        blocker_intercept = (
            kick_origin + kick_vector * blocker_intercept_dist_along_kick
        )

        blocker_distance = np.clip(
            np.linalg.norm(blocker_intercept - blocker_position) - EFF_BLOCK_WIDTH,
            a_min=0.0,
            a_max=np.inf,
        )

        blocker_time = np.abs(blocker_distance) / OPPONENT_SPEED

        # Doesn't include friction...oops?
        ball_time = np.linalg.norm(blocker_intercept - kick_origin) / kick_speed

        return blocker_time - ball_time

    def _kick_cost(
        self,
        point: np.array,
        kick_speed: float,
        kick_origin: np.array,
        world_state: stp.rc.WorldState,
    ):
        margins = [
            self._blocker_margin(kick_origin, point, kick_speed, blocker)
            for blocker in world_state.their_robots
        ]
        return -min(margins)

    def _find_target_point(
        self, world_state: stp.rc.WorldState, kick_speed: float
    ) -> np.ndarray:
        goal_y = world_state.field.length_m
        cost = 0

        ball_pos = world_state.ball.pos

        # Heuristic: limit where we kick if we're very wide
        xmin = -0.43
        xmax = 0.43
        if abs(ball_pos[0]) > 1:
            kick_extent = -1 / ball_pos[0]
            if kick_extent < 0:
                xmin = np.clip(kick_extent, a_min=xmin, a_max=0)
            elif kick_extent > 0:
                xmax = np.clip(kick_extent, a_min=0, a_max=xmax)

        try_points = [np.array([x, goal_y]) for x in np.arange(xmin, xmax, step=0.05)]

        cost, point = min(
            [
                (
                    self._kick_cost(
                        point, kick_speed, world_state.ball.pos, world_state
                    ),
                    point,
                )
                for point in try_points
            ],
            key=lambda x: x[0],
        )

        return point
