"""This module contains the interfaces ISituation, IAnalyzer and IPlaySelector."""

from abc import ABC, abstractmethod
from typing import Dict, Tuple

import stp.play
import stp.rc as rc


class ISituation(ABC):
    """Interface for a situation."""

    ...


class IAnalyzer(ABC):
    """Interface for situation analyzer."""

    @abstractmethod
    def analyze_situation(
        self, world_state: rc.WorldState, game_info: rc.GameInfo
    ) -> ISituation:
        """Returns the best situation for the current world state.
        :param world_state: The current state of the world.
        :param game_info: The information about the state of the game.
        :return: The best situation for the current world state.
        """
        ...


class IPlaySelector(ABC):
    """Interface for play selector."""

    @abstractmethod
    def select(self, world_state: rc.WorldState) -> Tuple[ISituation, stp.play.IPlay]:
        """Selects the best situation and play given given the current world state.
        :param world_state: The current state of the world.
        :return: A tuple of the best situation and best play.
        """
        ...
