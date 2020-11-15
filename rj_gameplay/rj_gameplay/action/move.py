"""This module contains the interface and action for move."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np

class IMove(action.IAction, ABC):

    def done(self) -> bool:
        pass

class Move(IMove):

    def __init__(self, point: np.ndarray)
        self.point = point

    def tick(self, ctx: action.Ctx) -> None:
        pass
