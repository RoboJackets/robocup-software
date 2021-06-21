from dataclasses import dataclass
from typing import List, Optional

import stp.rc as rc
import stp.tactic as tactic
import stp.role as role

import rj_gameplay.eval
import rj_gameplay.skill as skills
from rj_gameplay.skill import mark
import stp.skill as skill

import numpy as np

import stp.global_parameters as global_parameters

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
        self.prev_result = None

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:


        # TODO: make a better way to avoid assignment of goalie to other roles
        if world_state.game_info is not None:
            if robot.id == world_state.game_info.goalie_id:
                return 99

        # TODO: prevent gameplay crashing w/out this check
        if robot is None or self.enemy_to_mark is None: 
            return 99

        # TODO(#1669): Remove this once role assignment no longer assigns non-visible robots
        if not robot.visible:
            return 99  # float('inf') threw ValueError

        # TODO: use the convenience func in stp/role/ that has a stickiness for the last assignment
        # TODO: this is actually using a local var, not the param given
        # figure out how the param should be used
        # if self.prev_result is not None and self.prev_result.role is not None:
        #     if robot.id == self.prev_result.role.robot.id:
        #         # return 0
        #         pass

        return np.linalg.norm(robot.pose[0:2]-self.enemy_to_mark.pose[0:2]) / global_parameters.soccer.robot.max_speed

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
            for i in range(self.num_markers)
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
            # assign n closest enemies to respective skill and role costFn
            closest_enemies = get_closest_enemies_to_ball(self.num_markers, world_state)
            # for i in range(self.num_markers):
            for i in range(len(closest_enemies)):
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

        for mse in self.mark_list:
            result = role_results[mse]
            if result[0].is_filled():
                index = self.mark_list.index(mse)
                if index != -1:
                    self.cost_list[index].prev_result = result[0]

        return skills

    def is_done(self, world_state):
        # TODO: replace all similar is_done() with a .all() and generator expr
        # see https://www.w3schools.com/python/ref_func_all.asp
        for mark_skill in self.mark_list:
            if not mark_skill.skill.is_done(world_state):
                return False
        return True
