"""This module contains Actions of the STP(A) hierarchy."""

from abc import ABC, abstractmethod
import stp.rc as rc

class IAction(ABC):
    """Interface for actions."""
    
    def spin(self) -> None:
        """Spins the action"""
        pass

    @abstractmethod
    def done(self) -> bool:
        """Checks to see if the action is done running"""
        ...
