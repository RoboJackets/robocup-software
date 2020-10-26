"""Contains the ICapture interface."""

from abc import ABC, abstractmethod

import stp.role as role
import stp.skill as skill

import rj_gameplay.eval as eval


class ICapture(skill.ISkill, ABC):
    ...


class Capture(ICapture):
    def define(self):
        pass

    def create_request(self, recv_pass: eval.Pass) -> role.RoleRequest:
        pass
