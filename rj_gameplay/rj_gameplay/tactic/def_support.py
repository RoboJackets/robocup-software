"""Contains the DefSupport tactic.
"""

from dataclasses import dataclass
from typing import List, Optional

import stp.action as action
import stp.rc as rc
import stp.skill.stub as stub
import stp.tactic as tactic


class Skills(tactic.SkillsEnum):
    STUB = tactic.SkillEntry(stub.Stub)


class DefSupport(tactic.ITactic["Props"]):
    """Tactic that supports the ball carrier defensively."""

    @dataclass
    class Props:
        ...

    __slots__ = ["skills", "STUB"]

    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.STUB = self.skills.STUB

    def compute_props(self, prev_props: Optional[Props]) -> Props:
        pass

    def get_requests(
        self, world_state: rc.WorldState, props: Props
    ) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = tactic.RoleRequests()

        role_requests[self.STUB] = self.STUB.skill.create_request()

        return role_requests

    def tick(
        self, role_results: tactic.RoleResults, props: Props
    ) -> List[action.IAction]:
        return []
