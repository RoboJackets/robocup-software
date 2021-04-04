"""This module contains the interface and action for kick."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np
import stp.rc as rc

class IKick(action.IAction, ABC):
    
    def done(self) -> bool:
        pass

class Kick(IKick):
    """
    Kick action
    TODO: update with actions implementation
    """

    def __init__(self, point: np.ndarray):
        self.point = point
        self.count = -1
        #for stub

    def tick(self, robot: rc.Robot, ctx: action.Ctx) -> None:
        print('robot:', robot.id, 'kicking')
        self.count += 1

    def done(self) -> bool:
        return self.count == 1

    def fail(self):
        return False