from abc import abstractmethod, ABC
from typing import Dict, Type

import stp.rc as rc
import stp.play


class ISituation(ABC):
    """Interface for a situation."""

    @abstractmethod
    def is_applicable(self, world_state: rc.WorldState) -> bool:
        """Returns true if the current situation is applicable given the current world
        state."""
        ...

    @abstractmethod
    def score(self, world_state: rc.WorldState) -> float:
        """Returns the "goodness" of this situation given the current world state.
        [0-1]"""
        ...


PlayRegistry = Dict[ISituation, stp.play.IPlay]
