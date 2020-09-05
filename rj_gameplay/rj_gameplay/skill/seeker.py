"""Contains the ISeeker interface."""

from abc import ABC, abstractmethod

import stp.skill as skill
from stp import role as role


class ISeeker(skill.ISkill, ABC):
    @abstractmethod
    def seek(self, world_):


class Seeker(ISeeker):
    def define(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        pass

