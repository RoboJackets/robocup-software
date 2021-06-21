"""Module that contains NaiveRoleAssignment. """

from math import isfinite
import sys
from typing import List, Optional, Tuple

import numpy as np
import scipy.optimize
import stp.rc
import stp.role as role
import stp.role.assignment as assignment
from stp.role import RoleResult

SortedRequests = List[assignment.FlatRoleRequests]

# Define some big constant for "hard constraints".
INVALID_COST = sys.maxsize


class NaiveRoleAssignment(assignment.IRoleAssignment):
    """Naive implementation of role assignment that simply performs the Hungarian
    Algorithm (from scipy.optimize) on HIGH, then MEDIUM, then LOW priority in that
    order."""

    @staticmethod
    def get_sorted_requests(requests: assignment.FlatRoleRequests) -> SortedRequests:
        """Returns a list of FlatRoleRequests sorted in ascending priority order.
        :param requests: Flat list of requests.
        :return: List of FlatRoleRequests in sorted ascending priority order, ie.
        [LOW, MEDIUM, HIGH].
        """
        role_id: assignment.RoleId
        request: role.RoleRequest

        sorted_requests: SortedRequests = [
            {} for _ in range(role.Priority.NUM_PRIORITIES)
        ]

        for role_id, request in requests.items():
            sorted_requests[int(request.priority)][role_id] = request

        return sorted_requests

    @staticmethod
    def compute_costs_matrix(
        free_robots: np.ndarray,
        flat_requests: assignment.FlatRoleRequests,
        world_state: stp.rc.WorldState,
        prev_results: assignment.FlatRoleResults,
    ) -> np.ndarray:
        """Computes the n_free_robots x n_requests cost matrix corresponding to the
        passed in free robots and role requests.
        :param flat_requests: Role requests to compute cost matrix for.
        :param free_robots: Free robots to compute cost matrix for.
        :param world_state: Current world state.
        :param prev_results: The previous results.
        :return: The m x n cost matrix corresponding to the passed in free robots and
        role requests.
        """

        # Cost matrix size (n_free_robots, n_requests)
        robot_costs: np.ndarray = np.zeros((len(free_robots), len(flat_requests)))

        # Iterate over each role request.
        request: role.RoleRequest
        for request_idx, (role_id, request) in enumerate(flat_requests.items()):
            # For each role request, iterate over robots.
            for robot_idx, robot in enumerate(free_robots):
                # Get the previous result for this role_id, if available.
                prev_result: Optional[RoleResult] = prev_results.get(role_id, None)

                # If the constraints are not satisfied, set the cost to INVALID_COST
                # and continue.
                if not request.constraint_fn(robot, prev_result, world_state):
                    robot_costs[robot_idx, request_idx] = INVALID_COST
                    continue

                # Otherwise, record the cost.
                cost: float = request.cost_fn(robot, prev_result, world_state)

                # Throw an exception if the returned cost is not finite.
                if not isfinite(cost):
                    raise ValueError(
                        "Got a non-finite cost ({}) for request {} and robot {}".format(
                            cost, request, robot
                        )
                    )

                # Add to robot_costs.
                robot_costs[robot_idx, request_idx] = cost

        return robot_costs

    @staticmethod
    def assign_prioritized_roles(
        flat_requests: assignment.FlatRoleRequests,
        world_state: stp.rc.WorldState,
        free_robots: np.ndarray,
        prev_results: assignment.FlatRoleResults,
    ) -> Tuple[assignment.FlatRoleResults, np.ndarray]:
        """Assigns roles using the Hungarian algorithm.
        :param flat_requests: The role requests.
        :param world_state: The current state of the game.
        :param free_robots: The array of free robots that haven't been assigned yet.
        This list will be mutated. Array of stp.rc.Robot.
        :param prev_results: The previous results.
        :return: The results of the role assignment and the new free_robots after
        assignment.
        """

        # Create the flat results.
        flat_results: assignment.FlatRoleResults = {
            role_id: role.RoleResult.from_request(request)
            for role_id, request in flat_requests.items()
        }

        # Create a list of keys, so that we have a mapping from indices -> dict entries.
        keys_list: List[assignment.RoleId] = list(flat_requests.keys())

        # Compute the n x m cost matrix.
        robot_costs: np.ndarray = NaiveRoleAssignment.compute_costs_matrix(
            free_robots, flat_requests, world_state, prev_results
        )

        # Get the optimal assignment using the Hungarian algorithm.
        robot_ind: np.ndarray
        request_ind: np.ndarray
        robot_ind, request_ind = scipy.optimize.linear_sum_assignment(robot_costs)

        # Set the assigned robot for each request.
        for assignment_idx in range(request_ind.shape[0]):
            request_idx: int = request_ind[assignment_idx]
            robot_idx: int = robot_ind[assignment_idx]
            cost: float = robot_costs[robot_idx, request_idx]

            # If cost is equal to INVALID_COST, then it means that role assignment
            # failed. Throw an exception.
            if cost == INVALID_COST:
                raise RuntimeError("Hard constraints not satisfied!")

            robot: stp.rc.Robot = free_robots[robot_idx]
            role_id = keys_list[request_idx]

            flat_results[role_id].assign(robot, cost)

        # Remove the robots from free_robots.
        free_robots = np.delete(free_robots, robot_ind)

        return flat_results, free_robots

    @staticmethod
    def assign_roles(
        flat_requests: assignment.FlatRoleRequests,
        world_state: stp.rc.WorldState,
        prev_results: assignment.FlatRoleResults,
    ) -> assignment.FlatRoleResults:
        """Assigns roles.
        :param flat_requests: The role requests.
        :param world_state: The current state of the game.
        :param prev_results: The previous results.
        :return: The results of the role assignment.
        """
        # Collect high, medium and low priority requests, sorted in ascending priority
        # order.
        sorted_requests: SortedRequests = NaiveRoleAssignment.get_sorted_requests(
            flat_requests
        )

        # Assign roles for each priority from HIGH to LOW.
        requests_dict: assignment.FlatRoleRequests

        # Make a copy of rc.our_robots so that we can mutate it without mutating
        # rc.our_robots.
        free_robots = np.array(world_state.our_robots)

        flat_results: assignment.FlatRoleResults = {}

        # Iterate over requests from HIGH to LOW.
        for requests_dict in reversed(sorted_requests):
            # Actually perform the assignment using the Hungarian algorithm.
            (
                prioritized_results,
                free_robots,
            ) = NaiveRoleAssignment.assign_prioritized_roles(
                requests_dict, world_state, free_robots, prev_results
            )

            # Add the prioritized_results to flat_results.
            flat_results.update(prioritized_results)

        return flat_results
