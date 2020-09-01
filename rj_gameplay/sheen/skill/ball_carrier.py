from abc import ABC, abstractmethod
from typing import Optional, Type

import numpy as np

import sheen.skill as skill
from sheen import role as role


class Pass:
    __slots__ = ["passer", "receiver", "pt"]

    passer: int
    receiver: int
    pt: np.ndarray

    def __init__(self, passer: int, receiver: int, pt: np.ndarray):
        self.passer = passer
        self.receiver = receiver
        self.pt = pt


class IBallCarrier(skill.ISkill, ABC):
    @abstractmethod
    def get_pass(self) -> Optional[Pass]:
        ...


class BallCarrier(IBallCarrier):
    def define(self):
        pass

    def get_pass(self) -> Optional[Pass]:
        ...

    def create_request(self) -> role.RoleRequest:
        pass
