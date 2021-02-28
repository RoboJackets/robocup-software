"""This module contains the interface and action for pivot."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np


class IPivot(action.IAction, ABC):

    def done(self) -> bool:
        pass

class Pivot(IPivot):

    def __init__(self, pivot_point: np.ndarray, target_point: np.ndarray)
        self.pivot_point = pivot_point
        self.target_point = target_point


    def tick(self, ctx: action.Ctx) -> None:
        pass
