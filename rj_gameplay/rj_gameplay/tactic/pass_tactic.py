<<<<<<< HEAD
from dataclasses import dataclass
from typing import List, Optional, Any
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any
=======
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

import stp

from rj_gameplay.role import receiver, passer
import numpy as np

import stp.global_parameters as global_parameters

from rj_msgs.msg import RobotIntent

<<<<<<< HEAD
class PassToClosestReceiver(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    """

    def __init__(
        self,
        action_client_dict: Dict[Type[Any], List[Any]],
        target_point: Optional[np.ndarray] = None,
        passer_robot: rc.Robot = None,
    ):
        self.target_point = target_point
        self.passer_robot = passer_robot
        self.chosen_receiver = None

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None or self.target_point is None:
            return 99
        # TODO (#1669)
        if not robot.visible:
            return 99
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 99
        if self.chosen_receiver is not None and self.chosen_receiver.id == robot.id:
            return -99

        # always pick closest receiver
        raw_dist = np.linalg.norm(robot.pose[0:2] - self.target_point)
        return raw_dist / global_parameters.soccer.robot.max_speed

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class PasserCost(role.CostFn):
    """
    A cost function for how to choose a robot that will pass
    TODO: Implement a better cost function
    """

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
        if robot.has_ball_sense:
            return 0
        else:
            # closest to ball
            return np.linalg.norm(world_state.ball.pos - robot.pose[0:2])

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class PassToOpenReceiver(role.CostFn):
    """
    A cost function for how to choose a robot to pass to
    TODO: Implement a better cost function
    CURRENTLY NOT READY FOR USE
    """

    def __init__(
        self,
        action_client_dict: Dict[Type[Any], List[Any]],
        target_point: Optional[np.ndarray] = None,
        passer_robot: rc.Robot = None,
    ):
        self.target_point = target_point
        self.passer_robot = passer_robot
        self.chosen_receiver = None

    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        if robot is None or self.target_point is None:
            return 99
        # TODO (#1669)
        if not robot.visible:
            return 99
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 99
        if self.chosen_receiver is not None and self.chosen_receiver.id == robot.id:
            return -99

        # TODO: pick "most open" pass
        cost = 0
        for enemy in world_state.their_robots:
            cost -= 10 * np.linalg.norm(enemy.pose[0:2] - robot.pose[0:2])

        # TODO: should be dist in sec
        # raw_dist = np.linalg.norm(robot.pose[0:2] - self.target_point)
        # cost = cost + raw_dist
        return cost

        if robot is None or self.target_point is None:
            return 99
        # TODO (#1669)
        if not robot.visible:
            return 99
        if self.passer_robot is not None and robot.id == self.passer_robot.id:
            # can't pass to yourself
            return 99

        # TODO: pick "most open" pass
        cost = 0
        for enemy in world_state.their_robots:
            cost -= 10 * np.linalg.norm(enemy.pose[0:2] - robot.pose[0:2])

        # TODO: should be dist in sec
        # raw_dist = np.linalg.norm(robot.pose[0:2] - self.target_point)
        # cost = cost + raw_dist
        return cost

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


class Pass(tactic.ITactic):
    """
    A passing tactic which captures then passes the ball
    """

    def __init__(
        self,
        action_client_dict: Dict[Type[Any], List[Any]],
        target_point: np.ndarray,
        passer_cost: role.CostFn,
        receiver_cost: role.CostFn,
    ):
        self.target_point = target_point
        self.pivot_kick = tactic.SkillEntry(
            pivot_kick.PivotKick(
                robot=None, target_point=target_point, chip=False, kick_speed=4.0
            )
        )
        self.receive = tactic.SkillEntry(receive.Receive())
        self.receiver_cost = receiver_cost
        self.passer_cost = passer_cost

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def find_potential_receiver(self, world_state: rc.WorldState) -> rc.Robot:
        cost = float("inf")
        receive_robot = None
        for robot in world_state.our_robots:
            curr_cost = self.receiver_cost(robot, None, world_state)
            if curr_cost < cost:
                cost = curr_cost
                receive_robot = robot
        return receive_robot

    def get_requests(
        self, world_state: rc.WorldState, props
    ) -> List[tactic.RoleRequests]:
        """Checks if we have the ball and returns the proper request
        :return: A list of size 2 of role requests
        """
=======
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

from enum import Enum, auto


class State(Enum):
    INIT = auto()
    ACTIVE = auto()
    INIT_PASSER_CAPTURE = auto()
    PASSER_CAPTURE = auto()
    GET_RECEIVER = auto()
    INIT_EXECUTE_PASS = auto()
    EXECUTE_PASS = auto()
    AWAIT_PASSER_KICK = auto()
    PASS_IN_TRANSIT = auto()
    INIT_AWAIT_RECEIVE = auto()
    EXECUTE_RECEIVE = auto()
    AWAIT_RECEIVE = auto()
    DONE = auto()


class PassTactic(stp.tactic.Tactic):
    def __init__(self, world_state: stp.rc.WorldState):
        super().__init__(world_state)

        self._state = State.INIT

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        self.assigned_roles = []
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is passer.PasserRole:
                self.assigned_roles.append(role(robot))
            elif role is receiver.ReceiverRole:
                self.assigned_roles.append(role(robot))

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)
        """
        FSM
         - init: request passer
         - on filled req: init_roles > tick passer's capture
         - when passer ready to pass: request receiver
         - on filled req: init_roles > tick passer pass
         - on ball passed: tick receiver, release passer role
         - when receiver done: done
        """

        role_intents = []

        if self._state == State.INIT:
            # TODO: allow Plays to pass in this (and further below) cost fns,
            #       otherwise behavior is not easy to manipulate
            self._role_requests = [
                (
                    stp.role.cost.PickClosestToPoint(world_state.ball.pos),
                    passer.PasserRole,
                )
            ]
            self._needs_assign = True

            self._state = State.INIT_PASSER_CAPTURE

        elif self._state == State.INIT_PASSER_CAPTURE:
            # assumes play has given new role_requests
            self.init_roles(world_state)
            self._state = State.PASSER_CAPTURE

        elif self._state == State.PASSER_CAPTURE:
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            intent = passer_role.tick(world_state)

            role_intents = [(passer_role.robot.id, intent)]

            if passer_role.pass_ready:
                self._state = State.GET_RECEIVER

        elif self._state == State.GET_RECEIVER:
            self._role_requests = [
                (
                    stp.role.cost.PickClosestToPoint(world_state.ball.pos),
                    passer.PasserRole,
                ),
                (
                    stp.role.cost.PickClosestToPoint(world_state.field.their_goal_loc),
                    receiver.ReceiverRole,
                ),
            ]
            self._needs_assign = True

            self._state = State.INIT_EXECUTE_PASS
            # decided to get receiver early to allow more coordination in theory, currently not written in
            # TODO: evaluate whether this is a dumb idea or not

        elif self._state == State.INIT_EXECUTE_PASS:
            # one tick delay for play role assignment
            self._state = State.EXECUTE_PASS

        elif self._state == State.EXECUTE_PASS:
            # assumes play has given new role_requests
            self.init_roles(world_state)

            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            # TODO: create func to find good target point
            # TODO: should update receiver_role robot every tick in the role (see skill/capture.py)
            target_point = world_state.our_robots[receiver_role.robot.id].pose[0:2]
            passer_role.set_execute_pass(target_point)

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            self._state = State.AWAIT_PASSER_KICK

        elif self._state == State.AWAIT_PASSER_KICK:
            # TODO: these lines are a little ugly, any fix?
            passer_role = self.assigned_roles[0]
            receiver_role = self.assigned_roles[1]

            role_intents = [
                (passer_role.robot.id, passer_role.tick(world_state)),
                (receiver_role.robot.id, receiver_role.tick(world_state)),
            ]

            if passer_role.is_done(world_state):
                self._state = State.PASS_IN_TRANSIT

        elif self._state == State.PASS_IN_TRANSIT:
            receiver_role = self.assigned_roles[1]

            self._role_requests = [
                (
                    stp.role.cost.PickRobotById(receiver_role.robot.id),
                    receiver.ReceiverRole,
                )
            ]
            self._needs_assign = True

            self._state = State.INIT_AWAIT_RECEIVE

        elif self._state == State.INIT_AWAIT_RECEIVE:
            # one tick delay for play role assignment
            self._state = State.EXECUTE_RECEIVE

        elif self._state == State.EXECUTE_RECEIVE:
            # assumes play has given new role_requests
            self.init_roles(world_state)

            # TODO: these lines are a little ugly, any fix?
            receiver_role = self.assigned_roles[0]
            receiver_role.set_receive_pass()

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            self._state = State.AWAIT_RECEIVE

        elif self._state == State.AWAIT_RECEIVE:
            receiver_role = self.assigned_roles[0]

            role_intents = [(receiver_role.robot.id, receiver_role.tick(world_state))]

            if receiver_role.is_done(world_state):
                self._state = State.DONE
                # end FSM

        return role_intents

    @property
    def needs_assign(self):
        # style is wrong here: this convoluted way of returning self._needs_assign is necessary because we want to set it to False after the call, always
        ret = self._needs_assign == True  # noqa: E712
        self._needs_assign = False
        return ret

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return self._state == State.DONE
