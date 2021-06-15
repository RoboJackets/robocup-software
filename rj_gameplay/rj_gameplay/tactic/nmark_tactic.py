from dataclasses import dataclass
from typing import List, Optional

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import mark
import stp.skill as skill

import numpy as np

def get_closest_enemies_to_ball(num_enemies: int, world_state: rc.WorldState) -> List[rc.Robot]:
    ball_pt = world_state.ball.pos

    dist_to_enemies = {
        np.linalg.norm(ball_pt - robot.pose[0:2]): robot
        for robot in world_state.their_robots
    }

    # sort dict keys by dist (shortest first)
    # return enemies that correspond to n shortest dists
    return [dist_to_enemies[dist] for dist in sorted(dist_to_enemies.keys())[0:num_enemies]]

class marker_cost(role.CostFn):
    """Pick mark robots based on dist to the ball point
    """
    def __init__(self, enemy_to_mark: rc.Robot=None):
        self.enemy_to_mark = enemy_to_mark 

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: can role.CostFn be expanded to include more params?
        #       e.g. non-WorldState pt

        return np.linalg.norm(robot.pose[0:2]-self.enemy_to_mark.pose[0:2])

class NMarkTactic(tactic.ITactic):
    """Marks the n closest enemies to ball with the closest robots on our team to said enemies.
    """
    def __init__(self, n: int):
        self.num_markers = n

        # create empty mark SkillEntry for each robot
        self.mark_list = [
            tactic.SkillEntry(mark.Mark())
            for i in range(self.num_markers)
        ]

        # create cost func for each robot
        self.cost_list = [
            marker_cost()
            for _ in self.mark_list
        ]
        
    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, props
    ) -> List[tactic.RoleRequests]:
        """
        :return: role request for n markers
        """

        if world_state is not None and world_state.ball.visible:
            # this has to be here bc it needs world_state
            closest_enemies = get_closest_enemies_to_ball(self.num_markers, world_state)
            for i in range(self.num_markers):
                self.mark_list[i].skill.target_robot = closest_enemies[i]
                self.cost_list[i].enemy_to_mark = closest_enemies[i]

        # create RoleRequest for each SkillEntry
        role_requests = {
            self.mark_list[i]: [role.RoleRequest(role.Priority.LOW, False, self.cost_list[i])]
            for i in range(self.num_markers)
        }

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[tactic.SkillEntry]:
        """
        :return: skills for the number of markers assigned from the n markers
        """

        # create list of skills based on if RoleResult exists for SkillEntry
        skills = [
            mark_skill_entry
            for mark_skill_entry in self.mark_list
            if role_results[mark_skill_entry][0]
        ]

        return skills

    def is_done(self, world_state):
        # TODO: replace all similar is_done() with a .all() and generator expr
        # see https://www.w3schools.com/python/ref_func_all.asp
        for mark_skill in self.mark_list:
            if not mark_skill.skill.is_done(world_state):
                return False
        return True
