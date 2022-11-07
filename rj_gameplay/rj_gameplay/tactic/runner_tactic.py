import stp
from rj_msgs.msg import RobotIntent
from typing import List, Tuple
from rj_gameplay.role import runner_role


class RunnerTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, runner_id: int):
        super().__init__(world_state)
        self._role_requests.append((stp.role.cost.PickRobotById(runner_id), runner_role.RunnerRole))

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        # only has one role, but it's easier to copy-paste the structure
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is runner_role.RunnerRole:
                self.assigned_roles.append(role(robot))

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        # only has one role request, but it's easier to copy-paste the structure
        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            # TODO: figure out why this prevents sim crash
            if role.robot is not None:
                robot_intents.append((role.robot.id, role.tick(world_state)))
        return robot_intents
    
    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self.assigned_roles[0].is_done(world_state)