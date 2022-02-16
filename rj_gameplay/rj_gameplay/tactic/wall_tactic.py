<<<<<<< HEAD
"""Tactic to build a wall between mark pt (e.g. ball) and defense pt (e.g. goal)."""

from dataclasses import dataclass
from typing import List, Optional, Any
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any
=======
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

from rj_gameplay.role import dumb_move

<<<<<<< HEAD
from rj_gameplay.skill import move
=======
import stp
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15
import rj_gameplay.eval
import rj_gameplay.skill as skills

from rj_msgs.msg import PathTargetMotionCommand

import stp.skill as skill
import numpy as np

# TODO: replace w/ global param server
from stp.utils.constants import RobotConstants, BallConstants
import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent

# TODO: move this out of calculations file and into this tactic
from rj_gameplay.calculations import wall_calculations

MIN_WALL_RAD = None


<<<<<<< HEAD
class wall_cost(role.CostFn):
    """Cost function for role request."""

    def __init__(self, wall_pt: np.ndarray = None, scale: float = 1.0):

        self.wall_pt = wall_pt
        self.scale = scale

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None:
            return 9999

        wall_pt = np.array([0.0, 0.0]) if self.wall_pt is None else self.wall_pt

        # TODO(#1669): Remove this once role assignment no longer assigns non-visible robots
        if not robot.visible:
            return 9999  # float('inf') threw ValueError

        # TODO: fix goalie assignment issue the right way
        # if np.linalg.norm(robot.pose[0:2] - world_state.field.our_goal_loc) < MIN_WALL_RAD:
        #     return 9999
        switch_cost = 0
        if prev_result and prev_result.is_filled():
            switch_cost = 1 * (prev_result.role.robot.id != robot.id)

        # costs should be in seconds, not dist
        return (
            self.scale
            * np.linalg.norm(robot.pose[0:2] - wall_pt)
            / global_parameters.soccer.robot.max_speed
            + switch_cost
        )

    def unassigned_cost_fn(
        self,
        prev_result: Optional[role.RoleResult],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE

    # def switch_cost_fn(
    #     self,
    #     prev_result: Optional["RoleResult"],
    #     world_state: rc.WorldState,
    #     sticky_weight: float
    # ) -> float:

    #     return


class WallTactic(tactic.ITactic):
    def __init__(
        self,
        action_client_dict: Dict[Type[Any], List[Any]],
        priority=role.Priority.MEDIUM,
        cost_scale: float = 1.0,
    ):

        self._action_client_dict = action_client_dict

        # create move SkillEntry for every robot
        self.move_var = tactic.SkillEntry(move.Move(action_client_dict))

        # create empty cost_var (filled in get_requests)
        self.cost_var = wall_cost(scale=cost_scale)
        self.priority = priority

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, wall_pt, props
    ) -> List[tactic.RoleRequests]:
        """
        :return: A list of role requests for move skills needed
        """
        if world_state and world_state.ball.visible:
            self.move_var.skill.target_point = wall_pt
            self.move_var.skill.face_point = world_state.ball.pos
            robot = self.move_var.skill.robot
            self.cost_var.wall_pt = wall_pt

        # create RoleRequest for each SkillEntry
        role_requests = {
            self.move_var: [role.RoleRequest(self.priority, False, self.cost_var)]
            for _ in range(1)
        }

        return role_requests
=======
class WallTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState, num_wallers: int):
        super().__init__(world_state)

        self.num_wallers = num_wallers

        self.wall_pts = wall_calculations.find_wall_pts(self.num_wallers, world_state)

        # request closest robot every pt
        for pt in self.wall_pts:
            self._role_requests.append(
                (stp.role.cost.PickClosestToPoint(pt), dumb_move.DumbMove)
            )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            pt = self.wall_pts[i]
            if role is dumb_move.DumbMove:
                self.assigned_roles.append(role(robot, pt, world_state.ball.pos))
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        self.wall_pts = wall_calculations.find_wall_pts(self.num_wallers, world_state)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            wall_pt = self.wall_pts[i]
            robot_intents.append(
                (role.robot.id, role.tick(world_state, target_point=wall_pt))
            )
        return robot_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # wall never ends (until basic defense decides)
        return False
