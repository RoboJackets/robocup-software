from typing import List, Tuple

import stp.play as play
import stp.rc as rc
import stp.situation


class SituationAnalyzer:
    """Class that, given the current world state, performs situational analysis to
    determine the current situation, then returns the play for that situation."""

    __slots__ = ["_registry"]

    _registry: stp.situation.PlayRegistry

    def __init__(self, registry: stp.situation.PlayRegistry):
        self._registry = registry

    def select(
        self, world_state: rc.WorldState
    ) -> Tuple[stp.situation.ISituation, play.IPlay]:
        """Returns the best situation and play for the current world state.

        First, finds the most applicable situation we're in based on the world_state.
        Then, returns a tuple of the situation and the play registered for that
        situation.
        """
        # Get the list of applicable situations, ie situations whose constraints are
        # satisfied.
        applicable_situations: List[
            stp.situation.ISituation
        ] = self._applicable_situations(world_state)

        # If there aren't any applicable situations, then throw. The play registry
        # _should_ have at least one "fallback" play that is always applicable to any
        # situation.
        if len(applicable_situations) == 0:
            raise RuntimeError(
                "There are no applicable situations for the current world_state!"
            )

        # Get the scores of all applicable situations.
        situation_scores: List[float] = [
            situation.score(world_state) for situation in applicable_situations
        ]

        # Find the highest scoring situation from the list of applicable situations.
        highest_score: float = max(situation_scores)
        highest_score_idx: int = situation_scores.index(highest_score)

        # Get the situation and play.
        highest_score_situation = applicable_situations[highest_score_idx]
        situation_play: play.IPlay = self._registry[highest_score_situation]

        return highest_score_situation, situation_play

    def _applicable_situations(
        self, world_state: rc.WorldState
    ) -> List[stp.situation.ISituation]:
        """Returns the list of situations where is_applicable returns true."""
        return [
            situation
            for situation in self._registry.keys()
            if situation.is_applicable(world_state)
        ]
