"""Contains the PassOrShoot tactic. """

from typing import List, Optional

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic

import rj_gameplay.skill.ball_carrier as ball_carrier
import rj_gameplay.skill.capture as capture
import rj_gameplay.skill.seeker as seeker


class Skills(tactic.SkillsEnum):
    """SKills for PassOrShoot."""

    BALL_CARRIER = tactic.SkillEntry(ball_carrier.IBallCarrier)
    SEEKERS = tactic.SkillEntry(seeker.ISeeker)
    RECEIVER = tactic.SkillEntry(capture.ICapture)


class PassOrShoot(tactic.ITactic):
    """Tactic that controls one ball carrier and multiple seekers."""

    __slots__ = ["skills", "BALL_CARRIER", "RECEIVER", "SEEKERS"]

    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.BALL_CARRIER = self.skills.BALL_CARRIER
        self.RECEIVER = self.skills.RECEIVER
        self.SEEKERS = self.skills.SEEKERS

    def get_requests(
        self, prev_skills: tactic.SkillsDict, world_state: rc.WorldState
    ) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = tactic.RoleRequests()

        maybe_pass = self.get_pass(prev_skills)
        if maybe_pass:
            role_requests[self.RECEIVER] = [self.RECEIVER.skill.create_request()]

        role_requests[self.BALL_CARRIER] = [self.BALL_CARRIER.skill.create_request()]
        role_requests[self.SEEKERS] = self.SEEKERS.skill.create_requests(
            len(world_state.our_robots)
        )

        return role_requests

    def tick(self, role_results: tactic.RoleResults) -> List[action.IAction]:
        return []

    @staticmethod
    def get_pass(prev_skills: tactic.SkillsDict) -> Optional[ball_carrier.Pass]:
        """Returns the pass from IBallCarrier if it is in prev_skills, otherwise None.
        :param prev_skills: Skills that were run in the previous tick.
        :return: The pass from IBallCarrier if it is in prev_skills, otherwise None.
        """
        if Skills.BALL_CARRIER not in prev_skills:
            return None
        return prev_skills[Skills.BALL_CARRIER].get_pass()
