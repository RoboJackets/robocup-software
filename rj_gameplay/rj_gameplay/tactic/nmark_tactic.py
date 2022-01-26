from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp

from rj_gameplay.role import marker, capture_role
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent


def get_opponents_to_mark(world_state: stp.rc.WorldState, num_markers: int):
    ball_pt = world_state.ball.pos

    dist_to_opponents = {
        np.linalg.norm(ball_pt - robot.pose[0:2]):robot
        for robot in world_state.their_robots
    }
    return [dist_to_opponents[dist] for dist in sorted(dist_to_opponents.keys())[0:num_markers]]

class NMarkTactic(stp.tactic.Tactic): 
    def __init__(self, world_state: stp.rc.WorldState, num_markers: int):
        super().__init__(world_state)
        self.num_markers = num_markers

        self.opponents_to_mark = get_opponents_to_mark(world_state, self.num_markers)
        # self._role_requests.append(
        #     (stp.role.cost.PickClosestRobot(self.opponents_to_mark.pos)

        #     )
        for i in range(len(self.opponents_to_mark)):
            if i==0:
                self._role_requests.append(
                    (stp.role.cost.PickClosestRobot(self.opponents_to_mark[i].pose[:2]), capture_role.CaptureRole) 
                    )
            else:
                self._role_requests.append(
                    (stp.role.cost.PickClosestRobot(self.opponents_to_mark[i].pose[:2]), marker.MarkerRole) 
                    )
            
            


    
    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is capturer.CaptureRole or marker.MarkerRole:
                self.assigned_roles.append(role(robot))
            # elif role is marker.MarkerRole:
            #     self.assigned_roles.append(role(robot))


    def tick(self, world_state: stp.rc.WorldState):

        self.opponents_to_mark = get_opponents_to_mark(world_state, self.num_markers)

        if len(self._role_requests) != len(self.assigned_roles):
            self.init_roles(world_state)

        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            robot_intents.append(
                (role.robot.id, role.tick(world_state))
                )

        return robot_intents


        # if self._state == "init":
        #     self._role_requests = []

        # elif self._state == "execute_mark":
        #     marker_role.set_mark()
        #     self._role_requests = [
        #     (
        #         stp.role.cost.PickClosestRobot(world_state.ball.pos)
        #         ),
        #     (
        #         stp.role.cost.PickClosestRobot(world_state.pos))

        #     ]
        # elif self._state == "switch_mark":


        # elif self._state == "keep_marking":

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False
        #While on the defense play, it always returns False









# class NMarkTactic(tactic.ITactic):
#     """Marks the n closest enemies to ball with the closest robots on our team to said enemies."""

#     def __init__(self, n: int):
#         self.num_markers = n

#         # create empty mark SkillEntry for each robot
#         self.mark_list = [
#             tactic.SkillEntry(mark.Mark()) for i in range(self.num_markers)
#         ]

#         # create cost func for each robot
#         self.cost_list = [marker_cost() for _ in range(self.num_markers)]

#     def compute_props(self):
#         pass

#     def create_request(self, **kwargs) -> role.RoleRequest:
#         """Creates a sane default RoleRequest.
#         :return: A list of size 1 of a sane default RoleRequest.
#         """
#         pass

#     def get_requests(
#         self, world_state: rc.WorldState, props
#     ) -> List[tactic.RoleRequests]:
#         """
#         :return: role request for n markers
#         """

#         if world_state is not None and world_state.ball.visible:
#             # assign n closest enemies to respective skill and role costFn
#             closest_enemies = get_closest_enemies_to_ball(self.num_markers, world_state)
#             for i in range(len(closest_enemies)):
#                 self.mark_list[i].skill.target_robot = closest_enemies[i]
#                 self.cost_list[i].enemy_to_mark = closest_enemies[i]

#         # create RoleRequest for each SkillEntry
#         role_requests = {
#             self.mark_list[i]: [
#                 role.RoleRequest(role.Priority.LOW, False, self.cost_list[i])
#             ]
#             for i in range(self.num_markers)
#         }

#         return role_requests

#     def tick(
#         self, world_state: rc.WorldState, role_results: tactic.RoleResults
#     ) -> List[tactic.SkillEntry]:
#         """
#         :return: skills for the number of markers assigned from the n markers
#         """

#         # create list of skills based on if RoleResult exists for SkillEntry
#         skills = [
#             mark_skill_entry
#             for mark_skill_entry in self.mark_list
#             if role_results[mark_skill_entry][0]
#         ]

#         return skills

#     def is_done(self, world_state):
#         # TODO: replace all similar is_done() with a .all() and generator expr
#         # see https://www.w3schools.com/python/ref_func_all.asp
#         for mark_skill in self.mark_list:
#             if not mark_skill.skill.is_done(world_state):
#                 return False
#         return True
