"""This module contains the interface and action for capture."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np


class ICapture(action.IAction, ABC):

    def done(self) -> bool:
        pass

class Capture(ICapture):
    """
    Capture action
    TODO: update with actions implementation
    """

    def __init__(self):
        self.count = -1
        # For stub

    def tick(self, robot: int, ctx: action.Ctx):
        print('robot:', robot.id, 'is capturing')
        self.count += 1

    def done(self) -> bool:
        return self.count == 1

    def fail(self):
        return False
        