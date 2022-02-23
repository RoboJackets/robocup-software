import stp
from rj_gameplay.role import dumb_move
from typing import List, Tuple
from rj_msgs.msg import RobotIntent


class LineTactic(stp.tactic.Tactic):
    """Tactic for line up play that puts all six robots in a line on the left of the field."""

    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        # compute move points
        # TODO: make start on side of the field, so this Tactic is actually useful during penalty situations
        # access the length and the width of the field (minus border), and starts robots there
        start = (
            (world_state.field.width_m - world_state.field.border_m) / 2,
            (world_state.field.length_m - world_state.field.border_m) / 2,
        )
        dy = 0.5
        # TODO: make the # here a param instead of hardcoding for same reason as above TODO
        self.move_points = [(start[0], start[1] - i * dy) for i in range(6)]

        # request closest robot every pt
        for pt in self.move_points:
            # for some reason stp.role doesn't need to be imported?
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(pt), dumb_move.DumbMove)
            )

        # OR hardcode certain ids to go
        # for i, pt in enumerate(self.move_points):
        #     self._role_requests.append((stp.role.cost.PickRobotById(5-i), dumb_move.DumbMove))

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[Tuple[int, RobotIntent]]:
        # returns list of (robot_id, robot_intent)

        # if not self.assigned_roles:
        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        return [(role.robot.id, role.tick(world_state)) for role in self.assigned_roles]

    def is_done(
        self,
        world_state: stp.rc.WorldState,
    ) -> bool:
        return all([role.is_done(world_state) for role in self.assigned_roles])

    def init_roles(
        self,
        world_state: stp.rc.WorldState,
    ):
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.move_points[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))
