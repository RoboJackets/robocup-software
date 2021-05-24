""" This module contains data structures for the Skills level of STP.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Type, TypeVar

import stp.role as role


class ISkill(ABC):
    """ Interface for Skills. """

    @abstractmethod
    def tick(self) -> None:
        ...


