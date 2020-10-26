"""This module contains PurePlay."""

from abc import ABC
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar

import stp.action as action
import stp.play as play
import stp.rc as rc
import stp.role.assignment as assignment
import stp.tactic as tactic


class PurePlay(play.IPlay["Prop"], ABC):
    """A stateless play that uses the same tactics for the entire lifetime."""

    __slots__ = ["_role_assignment", "tactics"]

    @dataclass
    class Prop:
        tactic_props: Dict[Type[tactic.ITactic], Any] = {}

    _role_assignment: assignment.IRoleAssignment
    tactic: play.TacticsEnum

    def __init__(self, tactics: play.TacticsEnum, ctx: play.Ctx):
        """Creates a pure play.
        :param tactics: The TacticsEnum that defines all the tactics used.
        :param ctx: The play.Context that contains the role assignment.
        """
        self._role_assignment = ctx.role_assignment
        self.tactics = tactics

    def compute_props(self, prev_props: Optional[Prop]) -> Prop:
        if not prev_props:
            prev_props = PurePlay.Prop()

        # For each tactic, call compute_props on it.
        tactic_entry: play.TacticEntry
        for tactic_entry in self.tactics:
            tactic_type = type(tactic_entry.tactic)

            prev_prop = prev_props.tactic_props.get(tactic_type, None)
            prev_props[tactic_type] = tactic_entry.tactic.compute_props(prev_prop)

        return prev_props

    def tick(
        self,
        world_state: rc.WorldState,
        prev_results: assignment.FlatRoleResults,
        props: Prop,
    ) -> Tuple[assignment.FlatRoleResults, List[action.IAction]]:
        """Performs one "tick" of the specified play.

        This should:
            1. Collect all the role requests from the tactics
            2. Perform role assignment
            3. Gives each tactic its assigned roles and getting a list of skills.
            4. Return the list of skills obtained.
        :param world_state: The current state of the world.
        :param prev_results: The previous results of role assignment.
        :param props: Props from compute_props. This should be "const", as all
        mutations should happen in compute_props.
        :return: The list of skill to run.
        """

        # Collect the role requests.
        role_requests: play.RoleRequests = self.collect_role_requests(world_state)

        # Flatten from a dict of dicts to a single dict.
        flat_requests: assignment.FlatRoleRequests = play.flatten_requests(
            role_requests
        )

        # Perform role assignment.
        assignment_results: assignment.FlatRoleResults = (
            self._role_assignment.assign_roles(flat_requests, world_state, prev_results)
        )

        # Give each tactic its assigned roles and get the list of skills.
        actions = self.get_actions_from_tactics(assignment_results)

        # Return the role assignment results and the list of skills.
        return assignment_results, actions

    def collect_role_requests(self, world_state: rc.WorldState) -> play.RoleRequests:
        """Collects the role requests from each tactic.
        :param world_state: The current WorldState.
        :return: The collected play.RoleRequests.
        """
        role_requests: play.RoleRequests = play.RoleRequests()

        tactic_entry: play.TacticEntry
        for tactic_entry in self.tactics:
            tactic_requests = tactic_entry.tactic.get_requests(world_state, None)
            role_requests[type(tactic_entry.tactic)] = tactic_requests

        return role_requests

    def get_actions_from_tactics(
        self, flat_results: assignment.FlatRoleResults, prop: Prop
    ) -> List[action.IAction]:
        """Passes the roles assigned from role assignment to the correct tactic.
        :param flat_results: The results of role assignment.
        :param prop: The props of the play. This should be "const".
        :return: The list of skills for each tactic.
        """
        # Unnest the results.
        nested_results: play.RoleResults = play.unflatten_results(flat_results)

        actions: List[action.IAction] = []

        # For each tactic, tick it with the roles that it requested and add all the
        # returned actions to the list of actions.
        tactic_entry: play.TacticEntry
        for tactic_entry in self.tactics:
            tactic_type = type(tactic_entry.tactic)
            results: tactic.RoleResults = nested_results[tactic_type]
            actions.extend(tactic_entry.tactic.tick(results, prop[tactic_type]))

        return actions
