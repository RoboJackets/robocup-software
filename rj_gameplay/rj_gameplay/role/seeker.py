import stp
import numpy as np
from typing import List, Tuple
from rj_gameplay.skill import move
from rj_msgs.msg import RobotIntent
from stp.utils.constants import RobotConstants
from scipy.optimize import minimize


class SeekerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot, my_region) -> None:
        # TODO: type this header

        super().__init__(robot)
        self.move_skill = None
        self.target_point = None
        self._my_region = my_region
        self._target_point = None
        self._ticks_since_reassign = 0

    def get_open_point(self, world_state, region: List, centroid) -> np.ndarray:
        """ """
        # TODO: docstring

        # Heuristics to add complexity to minimize function
        min_to_centroid = lambda x: np.linalg.norm(x - centroid)
        min_to_ball = lambda x: np.linalg.norm(x - world_state.ball.pos)
        min_to_goal = lambda x: np.linalg.norm(x - world_state.field.their_goal_loc)

        # line-of-sight calculations
        # (LOS is maximized by making dot product of angle between unit vectors as close to 0 as possible)
        ball_dir = lambda pos: pos - world_state.ball.pos
        ball_dir_norm = lambda pos: ball_dir(pos) / np.linalg.norm(ball_dir(pos))
        opps_to_ball = [
            ball_dir_norm(opp_robot.pose[0:2]) for opp_robot in world_state.their_robots
        ]
        # only opponent robot that most blocks matters here
        # note that this is in radians
        max_los = lambda pos: max(
            np.dot(ball_dir_norm(pos), opp_dir) ** 2 for opp_dir in opps_to_ball
        )

        # linearly combine above
        heuristic = lambda x: 0 + 100 * max_los(x)

        result = minimize(
            heuristic,
            centroid,
            bounds=((region[0], region[1]), (region[2], region[3])),
            tol=1e-3,
            options={"maxiter": 1000, "disp": False},
        )

        # noise to add randomness to resultant point if needed
        x_noise = 0
        y_noise = 0

        bestpoint = np.array([result.x[0] + x_noise, result.x[1] + y_noise])

        return bestpoint

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume our_robot has ball on init. Then:
         - on init: get every point in the OFFENSE section of field away from their_robots at a certain distance and move there

        """

        centroid = np.array(
            [
                ((self._my_region[0] + self._my_region[1]) / 2),
                ((self._my_region[2] + self._my_region[3]) / 2),
            ]
        )

        # find target point w/in region
        if self.target_point is None:
            self.target_point = centroid

        # only reassign every so often so robot can reach target pt
        if self._ticks_since_reassign > 10:
            self.target_point = self.get_open_point(
                world_state, self._my_region, centroid
            )
            self._ticks_since_reassign = 0

        self._ticks_since_reassign += 1

        # assign move skill
        self.move_skill = move.Move(
            robot=self.robot,
            target_point=self.target_point,
            face_point=world_state.ball.pos,
        )

        intent = self.move_skill.tick(world_state)

        return intent

    def is_done(self, world_state) -> bool:
        return self.move_skill.is_done(world_state)
