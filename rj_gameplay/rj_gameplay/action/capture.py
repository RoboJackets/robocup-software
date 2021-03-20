"""This module contains the interface and action for capture."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np


class ICapture(action.IAction, ABC):
    def done(self) -> bool:
        pass


class Capture(ICapture):
    def tick(self, ctx: action.Ctx):
        pass
