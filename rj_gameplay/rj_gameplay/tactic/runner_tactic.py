from typing import List, Tuple

import numpy as np
import stp
from rj_msgs.msg import RobotIntent

from rj_gameplay.role import runner_role


class RunnerTactic(stp.tactic.Tactic):
    """
    In many ways, this tactic behaves similar to the DumbMove Tactic. Given a list of points, it will move to each of them. Since this is a tutorial tactic, we want only Robot 1 assigned to this tactic.

    :param world_state: the current state of gameplay
    :type world_state: stp.rc.WorldState
    :param runner_id: the robot id that will complete this task on init
    :type runner_id: int
    :param pts: an array of points that the robot will move to
    :type pts: np.ndarray
    """

    def __init__(self, world_state: stp.rc.WorldState, runner_id: int):
        # special case where we want robot 1 to be the runner, from init
        super().__init__(world_state)
        self._role_requests.append(
            (stp.role.cost.PickRobotById(runner_id), runner_role.Runner)
        )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        # only has one role, but it's easier to copy-paste the structure
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is runner_role.Runner:
                self.assigned_roles.append(role(world_state, robot))

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        # only has one role request, but it's easier to copy-paste the structure
        return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # special case: we know the only role is Runner, so we borrow that is_done()
        return self.assigned_roles[0].is_done(world_state)
