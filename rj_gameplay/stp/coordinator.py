"""This module contains the implementation of the coordinator."""
from typing import Any, Dict, Optional, Type

import stp.play
import stp.rc as rc
import stp.role.assignment as assignment
import stp.situation


class Coordinator:
    """The coordinator is responsible for using SituationAnalyzer to select the best
    play to run, calling tick() on the play to get the list of actions, then ticking
    all of the resulting actions."""

    __slots__ = [
        "_play_registry",
        "_play_selector",
        "_prev_situation",
        "_prev_play",
        "_prev_role_results",
        "_props",
    ]

    _play_selector: stp.situation.IPlaySelector
    _prev_situation: Optional[stp.situation.ISituation]
    _prev_play: Optional[stp.play.IPlay]
    _prev_role_results: assignment.FlatRoleResults
    _props: Dict[Type[stp.play.IPlay], Any]

    # TODO(1585): Properly handle type annotations for props instead of using Any.

    def __init__(self, play_selector: stp.situation.IPlaySelector):
        self._play_selector = play_selector

        self._prev_situation = None
        self._prev_play = None
        self._prev_role_results = {}

    def tick(self, world_state: rc.WorldState) -> None:
        """Performs 1 ticks of the STP system:
            1. Selects the best play to run given the passed in world state.
            2. Ticks the best play, collecting the list of actions to run.
            3. Ticks the list of actions.
        :param world_state: The current state of the world.
        """

        # Call situational analysis to see which play should be running.
        cur_situation, cur_play = self._play_selector.select(world_state)

        cur_play_type: Type[stp.play.IPlay] = type(cur_play)

        # Update the props.
        cur_play_props = cur_play.compute_props(self._props.get(cur_play_type, None))

        # Collect the list of actions from the play.
        new_role_results, actions = cur_play.tick(
            world_state, self._prev_role_results, cur_play_props
        )

        # Execute the list of actions.
        for action in actions:
            action.tick()

        # Update _prev_*.
        self._prev_situation = cur_situation
        self._prev_play = cur_play
        self._prev_role_results = new_role_results
        self._props[cur_play_type] = cur_play_props
