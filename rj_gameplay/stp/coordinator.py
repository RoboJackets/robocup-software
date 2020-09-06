"""This module contains the implementation of the coordinator."""
from typing import Optional

import stp.play
import stp.rc as rc
import stp.role.assignment as assignment
import stp.situation
import stp.situation.analyzer as analyzer


class Coordinator:
    """The coordinator is responsible for using SituationAnalyzer to select the best
    play to run, calling tick() on the play to get the list of actions, then ticking
    all of the resulting actions."""

    __slots__ = [
        "_play_registry",
        "_analyzer",
        "_prev_situation",
        "_prev_play",
        "_prev_role_results",
    ]

    _play_registry: stp.situation.PlayRegistry
    _analyzer: analyzer.SituationAnalyzer
    _prev_situation: Optional[stp.situation.ISituation]
    _prev_play: Optional[stp.play.IPlay]
    _prev_role_results: assignment.FlatRoleResults

    def __init__(self, play_registry: stp.situation.PlayRegistry):
        self._play_registry = play_registry
        self._analyzer = analyzer.SituationAnalyzer(self._play_registry)

        self._prev_situation = None
        self._prev_play = None
        self._prev_role_results = {}

    def tick(self, world_state: rc.WorldState) -> None:
        # Call situational analysis to see which play should be running.
        cur_situation, cur_play = self._analyzer.select(world_state)

        # Collect the list of actions from the play.
        new_role_results, actions = cur_play.tick(world_state, self._prev_role_results)

        # Execute the list of actions.
        for action in actions:
            action.tick()

        # Update _prev_*.
        self._prev_situation = cur_situation
        self._prev_play = cur_play
        self._prev_role_results = new_role_results
