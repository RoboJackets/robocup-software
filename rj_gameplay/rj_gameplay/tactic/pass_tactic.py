from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar

import stp

from rj_gameplay.role import receiver, passer
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent


class PassTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        # TODO: make FSM class (or at least use enum instead of str literals)
        self._state = "init"

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is passer.PasserRole:
                self.assigned_roles.append(role(robot))
            elif role is receiver.ReceiverRole:
                self.assigned_roles.append(role(robot))

    def tick(self, world_state: stp.rc.WorldState):
        """
        FSM
         - init: request passer
         - on filled req: init_roles > tick passer's capture
         - when passer ready to pass: request receiver
         - on filled req: init_roles > tick passer pass
         - on ball passed: tick receiver, release passer role
         - when receiver done: done
        """
        # make this the default print behavior
        # print("tactic state:", self._state)

        role_intents = []

        if self._state == "init":
            self._role_requests = [
                (
                    stp.role.cost.PickClosestRobot(world_state.ball.pos),
                    passer.PasserRole,
                )
            ]
            self._needs_assign = True

            self._state = "init_passer_capture"

        elif self._state == "init_passer_capture":
            # assumes play has given new role_requests
            self.init_roles(world_state)
            self._state = "passer_capture"

        elif self._state == "passer_capture":
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            assert isinstance(passer_role, passer.PasserRole)
            intent = passer_role.tick(world_state)

            role_intents = [(passer_role.robot.id, intent)]

            if passer_role.pass_ready:
                self._state = "get_receiver"

        elif self._state == "get_receiver":
            self._role_requests = [
                (
                    stp.role.cost.PickClosestRobot(world_state.ball.pos),
                    passer.PasserRole,
                ),
                (
                    stp.role.cost.PickClosestRobot(world_state.field.their_goal_loc),
                    receiver.ReceiverRole,
                ),
            ]
            self._needs_assign = True

            self._state = "init_execute_pass"
            # decided to get receiver early to allow more coordination in theory, currently not written in
            # TODO: evaluate whether this is a dumb idea or not

        elif self._state == "init_execute_pass":
            # one tick delay for play role assignment
            self._state = "execute_pass"

        elif self._state == "execute_pass":
            # assumes play has given new role_requests
            self.init_roles(world_state)

            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            assert isinstance(passer_role, passer.PasserRole)
            receiver_role = self.assigned_roles[1]
            assert isinstance(receiver_role, receiver.ReceiverRole)

            # TODO: create func to find good target point
            # TODO: should update receiver_role robot every tick in the role (see skill/capture.py)
            target_point = world_state.our_robots[receiver_role.robot.id].pose[0:2]
            passer_role.set_execute_pass(target_point)

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            self._state = "await_pass"

        elif self._state == "await_pass":
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            assert isinstance(passer_role, passer.PasserRole)
            receiver_role = self.assigned_roles[1]
            assert isinstance(receiver_role, receiver.ReceiverRole)

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            if passer_role.is_done(world_state):
                self._state = "pass_incoming"

        elif self._state == "pass_incoming":
            receiver_role = self.assigned_roles[1]
            assert isinstance(receiver_role, receiver.ReceiverRole)

            self._role_requests = [
                (
                    stp.role.cost.PickRobotById(receiver_role.robot.id),
                    receiver.ReceiverRole,
                )
            ]
            self._needs_assign = True

            self._state = "init_await_receive"

        elif self._state == "init_await_receive":
            # one tick delay for play role assignment
            self._state = "execute_receive"

        elif self._state == "execute_receive":
            # assumes play has given new role_requests
            self.init_roles(world_state)

            # TODO: these lines are a little ugly, any fix?
            receiver_role = self.assigned_roles[0]
            assert isinstance(receiver_role, receiver.ReceiverRole)
            receiver_role.set_receive_pass()

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            self._state = "await_receive"

        elif self._state == "await_receive":
            receiver_role = self.assigned_roles[0]
            assert isinstance(receiver_role, receiver.ReceiverRole)

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            if receiver_role.is_done(world_state):
                self._state = "done"
                # end FSM

        return role_intents

    @property
    def needs_assign(self):
        # maybe issues with one-tick flag?
        ret = self._needs_assign == True
        self._needs_assign = False
        return ret

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == "done"
