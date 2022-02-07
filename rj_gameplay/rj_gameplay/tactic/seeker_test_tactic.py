import stp

from rj_gameplay.role import seeker

from rj_msgs.msg import RobotIntent

from typing import List, Tuple

class SeekerTactic(stp.tactic.Tactic):
    """Wrapper for the Seeker Role that handles assigning said role to whichever Robots are the seekers."""

    def __init__(self, world_state: stp.rc.WorldState):
        """Special case where we want only robot 0 to be goalie, from init."""
        super().__init__(world_state)

        # TODO: rather than passing in hardcoded goalie id on init, gameplay node should
        #       have a set robot and pass that (in case robot 0 is ejected or broken)
        self.state = "init"
    
    def init_roles(self, world_state: stp.rc.WorldState):
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is seeker.SeekerRole:
                self.assigned_roles.append(role(robot))
    
    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)
        
        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            robot_intents.append((role.robot.id, role.tick(world_state)))
        return robot_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == "done"

