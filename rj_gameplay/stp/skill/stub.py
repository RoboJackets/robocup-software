"""This module contains the Stub skill for ISkill."""

import stp.skill as skill
import stp.role as role


class Stub(skill.ISkill):
    """Stub skill that does nothing."""

    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
        pass
