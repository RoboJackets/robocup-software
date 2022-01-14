import stp.play as play
import stp.tactic as tactic

from rj_gameplay.tactic import wall_tactic, nmark_tactic, goalie_tactic, move_tactic
import stp.role
from stp.role.assignment.naive import NaiveRoleAssignment
import stp.rc
from typing import Dict, List, Tuple, Type
from rj_gameplay.calculations import wall_calculations

import stp.role.cost
from rj_msgs.msg import RobotIntent


class BasicDefense(play.Play):
    """For when we don't have the ball and are trying to stop the opponent from scoring."""

    def __init__(self):
        super().__init__()

    def tick(
        self,
        world_state: stp.rc.WorldState,
    ) -> List[RobotIntent]:

        # calculate wall pts
        num_wallers = 5
        self.wall_pts = wall_calculations.find_wall_pts(num_wallers, world_state)

        # if no tactics created, assign roles and create them
        if world_state is not None and not self.prioritized_tactics:
            # TODO: figure out better way to share wall pts than calculations file
            #       maybe a shared wall class? but wall tactic must remain 1:1 for roles

            # 1 goalie, 5 wallers (for now)
            # TODO: break nmark into single-robot mark tactics

            self.prioritized_role_requests = [
                (
                    goalie_tactic.GoalieTactic,
                    stp.role.cost.PickRobotById(world_state.goalie_id),
                )
            ]
            for pt in self.wall_pts:
                self.prioritized_role_requests.append(
                    (wall_tactic.WallTactic, stp.role.cost.PickClosestRobot(pt))
                )
            self.assign_roles(world_state)

        # return robot intents from assigned tactics back to gameplay node
        return self.get_robot_intents(world_state)

    def init_new_tactics(
        self, assigned_robots: List[stp.rc.Robot], world_state: stp.rc.WorldState
    ) -> None:

        wall_ct = 0
        for role_request, robot, pt in zip(
            self.prioritized_role_requests, assigned_robots, self.wall_pts
        ):
            role, cost_fn = role_request

            new_tactic = None
            # TODO: this is bad, shouldn't have to check types of tactics imo
            # although perhaps this is fine since the play has to fill prioritized_roles dynamically anyhow, indicating it knows what roles to expect and what order
            if role is wall_tactic.WallTactic:
                # TODO: figure out how to coordinate multiple robots in less janky way
                new_tactic = role(robot, wall_ct)
                new_tactic.pass_wall_pts(self.wall_pts)
                wall_ct += 1
            elif role is goalie_tactic.GoalieTactic:
                new_tactic = role(robot)

            if new_tactic is not None:
                self.prioritized_tactics.append(new_tactic)
