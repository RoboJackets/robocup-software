"""Contains the DefSupport tactic.
"""

from typing import List

import stp.tactic as tactic
import stp.skill.stub as stub
import stp.action as action


class Skills(tactic.SkillsEnum):
    STUB = tactic.SkillEntry(stub.Stub)


class DefSupport(tactic.ITactic):
    """Tactic that supports the ball carrier defensively."""

    __slots__ = ["skills", "STUB"]

    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.STUB = self.skills.STUB

    def get_requests(self, prev_skills: tactic.SkillsDict) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = tactic.RoleRequests()

        role_requests[self.STUB] = self.STUB.skill.create_request()

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[action.IAction]:
        return []
