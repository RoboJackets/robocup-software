"""This module contains the interface and action for pivot."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc


class IPivot(action.IAction, ABC):

    def done(self) -> bool:
        pass

class Pivot(IPivot):
    """
    Pivot Skill
    TODO: update with actions implementation
    """
    def __init__(self, pivot_point: np.ndarray, target_point: np.ndarray):
        self.pivot_point = pivot_point
        self.target_point = target_point
        self.count = -1
        #for stub


    def tick(self, robot: rc.Robot, ctx: action.Ctx) -> None:
       print('robot:', robot.id, 'pivoting')
       self.count += 1

    def done(self) -> bool:
        return self.count == 1

    def fail(self):
        return False
        