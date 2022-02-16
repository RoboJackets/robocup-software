import stp

from dataclasses import dataclass
from typing import Dict, Type, Tuple, List, Any
from rj_gameplay.role import goalie_role

from rj_msgs.msg import RobotIntent

class GoalieTactic(stp.tactic.Tactic):
    """Wrapper for the Goalie Role that handles assigning said role to whichever Robot is our goalie."""

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], world_state: stp.rc.WorldState, goalie_id: int):
        """Special case where we want only robot 0 to be goalie, from init."""
        super().__init__(action_client_dict, world_state)
        # TODO: rather than passing in hardcoded goalie id on init, gameplay node should
        #       have a set robot and pass that (in case robot 0 is ejected or broken)
        self._role_requests.append(
            (stp.role.cost.PickRobotById(goalie_id), goalie_role.GoalieRole)
        )

    def init_roles(self, action_client_dict: Dict[Type[Any], List[Any]], world_state: stp.rc.WorldState) -> None:
        # only has one role, but it's easier to copy-paste the structure
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is goalie_role.GoalieRole:
                self.assigned_roles.append(role(action_client_dict, robot))

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
            robot_intents.append((role.robot.id, role.tick(world_state)))
        return robot_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # special case: we know the only role is Goalie, so we borrow that is_done()
        return self.assigned_roles[0].is_done(world_state)
