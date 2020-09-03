from typing import List
import numpy as np


class Robot:
    """Robot. Pose: [x, y, theta]. Twist: [dx, dy, dtheta]."""

    __slots__ = ["id", "pose", "twist", "has_ball"]
    id: int
    pose: np.ndarray
    twist: np.ndarray
    has_ball: bool

    def __init__(
        self, shell_id: int, pose: np.ndarray, twist: np.ndarray, has_ball: bool
    ):
        """
        :param shell_id: Shell id of the robot.
        :param pose: Pose of the Robot. [x, y, theta].
        :param twist: Twist of the robot. [dx, dy, dtheta].
        :param has_ball: Whether the robot has the ball or not.
        """
        self.id = shell_id
        self.pose = pose
        self.twist = twist
        self.has_ball = has_ball

    def __repr__(self) -> str:
        return "Robot(id:{}, pose:{}, twist:{}, has_ball:{})".format(
            self.id, self.pose, self.twist, self.has_ball
        )


class Ball:
    """Ball."""

    __slots__ = ["pos", "vel"]

    pos: np.ndarray
    vel: np.ndarray

    def __init__(self, pos: np.ndarray, vel: np.ndarray):
        self.pos = pos
        self.vel = vel

    def __repr__(self) -> str:
        return "Ball(pos:{}, vel:{})".format(self.pos, self.vel)


class WorldState:
    """Current state of the game."""

    __slots__ = ["our_robots", "their_robots", "ball"]

    def __init__(self, our_robots: List[Robot], their_robots: List[Robot], ball: Ball):
        self.our_robots = our_robots
        self.their_robots = their_robots
        self.ball = ball
