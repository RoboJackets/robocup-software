import stp.skill as skill
from stp import role as role


class Stub(skill.ISkill):
    """Stub skill that does nothing."""

    def define(self):
        pass

    def create_request(self) -> role.RoleRequest:
        pass
