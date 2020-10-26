"""This module contains the Stub skill for ISkill."""

import stp.role as role
import stp.skill as skill


class Stub(skill.ISkill):
    """Stub skill that does nothing."""

    def create_request(self, **kwargs) -> role.RoleRequest:
        pass
