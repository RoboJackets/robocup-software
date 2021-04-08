"""This module contains Actions of the STP(A) hierarchy."""

from abc import ABC, abstractmethod
from typing import Dict, MutableMapping, Type, TypeVar


class IAction(ABC):
    """Interface for actions."""

    @abstractmethod
    def tick(self) -> None:
        """Ticks the action."""
        ...

