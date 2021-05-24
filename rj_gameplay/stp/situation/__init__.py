"""This module contains the interfaces ISituation, IAnalyzer and IPlaySelector."""

from abc import ABC, abstractmethod
from typing import Dict, Tuple

import stp.play
import stp.rc as rc


class IPlaySelector(ABC):
    """Interface for play selector."""

    @abstractmethod
    def select(self, world_state: rc.WorldState) -> stp.play.IPlay:
        """Selects the best play given given the current world state.
        :param world_state: The current state of the world.
        :return: The best play.
        """
        ...
