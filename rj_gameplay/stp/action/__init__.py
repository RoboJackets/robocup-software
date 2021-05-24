"""This module contains Actions of the STP(A) hierarchy."""

from abc import ABC, abstractmethod
from typing import Dict, MutableMapping, Type, TypeVar
import stp.rc as rc


class IAction(ABC):
    """Interface for actions."""

    @abstractmethod 
    def tick(self, intent) -> None:
        """Modifies the passed RobotIntent with the relavant fields"""
        pass

class IFiniteAction(IAction, ABC):
    """Interface for actions that end"""

    @abstractmethod
    def is_done(self, world_state) -> bool:
        """Checks to see if the action is done running"""
        ...

class Ctx:
    """Context for actions, dummy for now"""
    ...