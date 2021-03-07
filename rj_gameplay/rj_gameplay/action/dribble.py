"""This module contains the interface and action for dribble."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.action as action
import numpy as np


class IDribble(action.IAction, ABC):

    def done(self) -> bool:
        pass

class Drible(IDribble):

    def __init__(self):
        pass


    def tick(self, ctx: action.Ctx) -> None:
        pass
