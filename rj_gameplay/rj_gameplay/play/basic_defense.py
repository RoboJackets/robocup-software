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

        # if no tactics created, assign roles and create them
        if world_state is not None and not self.prioritized_tactics:
            # TODO: figure out better way to share wall pts than calculations file
            #       maybe a shared wall class? but wall tactic must remain 1:1 for roles
            #
            # OR change role assignment to accept multi-tactic roles and output how many robots it needs?
            # would make some passing things more logically separated, theoretically

            # 1 goalie, 5 wallers (for now)
            # TODO: break nmark into single-robot mark tactics

            # pre-calculate wall points and store in numpy array
            num_wallers = 5
            self.wall_pts = wall_calculations.find_wall_pts(num_wallers, world_state)

            self.prioritized_role_requests = [
                (
                    goalie_tactic.GoalieTactic,
                    stp.role.cost.PickRobotById(world_state.goalie_id),
                )
            ]
            for pt in self.wall_pts:
                self.prioritized_role_requests.append(
                    (move_tactic.MoveTactic, stp.role.cost.PickClosestRobot(pt))
                )
            self.assign_roles(world_state)

        # TODO: fix wall tactic to change its own point based on ball pos, without feeding info via Play

        # return robot intents from assigned tactics back to gameplay node
        return self.get_robot_intents(world_state)

    def init_new_tactics(
        self, assigned_robots: List[stp.rc.Robot], world_state: stp.rc.WorldState
    ) -> None:

        for role_request, robot, wall_pt in zip(
            self.prioritized_role_requests, assigned_robots, self.wall_pts
        ):
            role, cost_fn = role_request

            new_tactic = None
            # TODO: this is bad, shouldn't have to check types of tactics imo
            # although perhaps this is fine since the play has to fill prioritized_roles dynamically anyhow, indicating it knows what roles to expect and what order
            if role is move_tactic.MoveTactic:
                new_tactic = role(robot, wall_pt, world_state.ball.pos)
            elif role is goalie_tactic.GoalieTactic:
                new_tactic = role(robot)

            if new_tactic is not None:
                self.prioritized_tactics.append(new_tactic)
