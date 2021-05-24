"""This module contains the interface and action for move."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import stp.rc as rc
import numpy as np

class IMove(action.IAction, ABC):

    def done(self) -> bool:
        pass

class Move(IMove):
    """
    Move Action
    TODO: update with actions implementation
    """
    def __init__(self, point: np.ndarray):
        self.point = point

    def tick(self, robot: rc.Robot, ctx: action.Ctx=None) -> None:
        print('robot:', robot.id, 'moving to', self.point)
        # print for stub
