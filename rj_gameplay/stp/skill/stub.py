"""This module contains the Stub skill for ISkill."""

import stp.role as role
import stp.skill as skill


class Stub(skill.ISkill):
    """Stub skill that does nothing."""

    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
        pass
