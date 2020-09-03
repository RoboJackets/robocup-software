from typing import List, Dict

import stp.play as play
import stp.tactic as tactic
import stp.skill as skill
import stp.role.assignment as assignment


class PurePlay(play.IPlay):
    """A stateless play that uses the same tactics for the entire lifetime."""

    __slots__ = ["tactics"]

    def __init__(self, tactics: play.TacticsEnum):
        self.tactics = tactics

    def tick(self) -> List[skill.ISkill]:
        role_requests = self.collect_role_requests()

        return []

    def collect_role_requests(self) -> play.RoleRequests:
        """Collects the role requests from each tactic.
        :return: The collected play.RoleRequests.
        """
        prev_skills: tactic.SkillsDict = tactic.SkillsDict()
        role_requests: play.RoleRequests = play.RoleRequests()
        for tactic_entry in self.tactics:
            tactic_requests = tactic_entry.tactic.get_requests(prev_skills)
            role_requests[type(tactic_entry.tactic)] = tactic_requests

        return role_requests

    def get_skills(self) -> List[skill.ISkill]:
        ...


# skills = []
# for tactic_entry in self.tactics:
#     skills += tactic_entry.tactic.
#
# return skills
