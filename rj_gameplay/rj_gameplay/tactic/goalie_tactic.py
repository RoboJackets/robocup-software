import stp

from rj_gameplay.role import goalie_role

from rj_msgs.msg import RobotIntent


class GoalieTactic(stp.tactic.Tactic):
    """Single-robot Tactic that serves as a Goalie Role wrapper."""

    def __init__(self, world_state: stp.rc.WorldState, goalie_id: int):
        """Special case where we want only robot 0 to be goalie, from init."""
        super().__init__(world_state)

        self._role_requests.append(
            (stp.role.cost.PickRobotById(goalie_id), goalie_role.GoalieRole)
        )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        # only has one role, but it's easier to copy-paste the structure
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is goalie_role.GoalieRole:
                self.assigned_roles.append(role(robot))

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
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
