import stp.tactic as tactic
import stp.skill.stub as stub


class Skills(tactic.SkillsEnum):
    STUB = tactic.SkillEntry(stub.Stub)


class DefSupport(tactic.ITactic):
    """Tactic that supports the ball carrier defensively."""

    __slots__ = ["skills", "STUB"]

    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.STUB = self.skills.STUB

    def define(self, prev_skills: tactic.SkillsDict) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = tactic.RoleRequests()

        role_requests[self.STUB] = self.STUB.skill.create_request()

        return role_requests
