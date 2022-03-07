import stp
import numpy as np
from typing import List, Tuple
from rj_gameplay.skill import move
from rj_msgs.msg import RobotIntent
from stp.utils.constants import RobotConstants


class SeekerRole(stp.role.Role):
    def __init__(self, robot: stp.rc.Robot, my_region) -> None:
        # TODO: type this header

        super().__init__(robot)
        self.move_skill = None
        self._my_region = my_region
        self._target_point = None

    def get_opp_in_region(
        self, region: Tuple[float, float], world_state: stp.rc.WorldState
    ) -> List[Tuple[float, float]]:
        opp_in_region = []
        # check to see if any opp robot in the region called
        for opp_robot in world_state.their_robots:
            x, y = opp_robot.pose[0:2]
            in_x_bounds = x > region[0] and x < region[2]
            in_y_bounds = y > region[1] and y < region[3]
            if in_x_bounds and in_y_bounds:
                opp_in_region.append(opp_pos)
            else:
                break

        return opp_in_region

    def get_open_point(self, region: List, opp_in_region: List) -> np.ndarray:
        # TODO: research/ask about algorithm that does this task without a large time complexity
        for x in range(region[0], region[2], RobotConstants.RADIUS):
            for y in range(region[1], region[3], RobotConstants.RADIUS):
                point = np.array((x, y))
                for pos in opp_in_region:
                    SAG_DIST = RobotConstants.RADIUS * 0.5
                    distvec = pos - point
                    dist = np.linalg.norm(distvec)
                    if dist > SAG_DIST:
                        bestpoint = point
                    else:
                        break
                break
            break

        return bestpoint

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        """
        Assume our_robot has ball on init. Then:
         - on init: get every point in the OFFENSE section of field away from their_robots at a certain distance and move there

        """

        # find pos of opps in region
        opp_in_region = self.get_opp_in_region(self._my_region, world_state)

        # find target point w/in region
        # (centroid if no opp in region)
        if len(opp_in_region) == 0:
            centroid = np.array(
                [
                    ((self._my_region[0] + self._my_region[2]) / 2),
                    ((self._my_region[1] + self._my_region[3]) / 2),
                ]
            )
            print(centroid)
            self.target_point = centroid
        else:
            self.target_point = self.get_open_point(self._my_region, opp_in_region)

        # print(self.target_point)

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
