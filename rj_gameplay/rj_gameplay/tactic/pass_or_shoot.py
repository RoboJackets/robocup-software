"""Contains the PassOrShoot tactic. """

from dataclasses import dataclass
from typing import List, Optional

import stp.action as action
import stp.rc as rc
import stp.tactic as tactic
from stp.role import RoleResult

import rj_gameplay.eval as eval
import rj_gameplay.skill.ball_carrier as ball_carrier
import rj_gameplay.skill.capture as capture
import rj_gameplay.skill.seeker as seeker
from rj_gameplay.skill.ball_carrier import IBallCarrier


class Skills(tactic.SkillsEnum):
    """Skills for PassOrShoot."""

    BALL_CARRIER = tactic.SkillEntry(ball_carrier.IBallCarrier)
    SEEKERS = tactic.SkillEntry(seeker.ISeeker)
    RECEIVER = tactic.SkillEntry(capture.ICapture)


@dataclass
class Props:
    """Props (state) for PassOrShoot."""

    maybe_pass: Optional[eval.Pass] = None


class PassOrShoot(tactic.ITactic[Props]):
    """Tactic that controls one ball carrier and multiple seekers."""

    __slots__ = ["skills", "BALL_CARRIER", "RECEIVER", "SEEKERS"]

    def __init__(self, ctx: tactic.Ctx):
        self.skills = Skills(ctx.skill_factory)

        self.BALL_CARRIER = self.skills.BALL_CARRIER
        self.RECEIVER = self.skills.RECEIVER
        self.SEEKERS = self.skills.SEEKERS

    def compute_props(self, prev_props: Optional[Props]) -> Props:
        return Props()

    def get_requests(
        self, world_state: rc.WorldState, props: Props
    ) -> tactic.RoleRequests:
        role_requests: tactic.RoleRequests = tactic.RoleRequests()

        if props.maybe_pass:
            role_requests[self.RECEIVER] = [self.RECEIVER.skill.create_request()]

        role_requests[self.BALL_CARRIER] = [self.BALL_CARRIER.skill.create_request()]
        role_requests[self.SEEKERS] = self.SEEKERS.skill.create_requests(
            len(world_state.our_robots)
        )

        return role_requests

    def tick(
        self, role_results: tactic.RoleResults, props: Props
    ) -> List[action.IAction]:

        receiver_role: RoleResult = role_results[self.RECEIVER][0]
        ball_carrier_role: RoleResult = role_results[self.BALL_CARRIER][0]
        seeker_roles: List[RoleResult] = role_results[self.SEEKERS]

        actions: List[action.IAction] = []

        assert ball_carrier_role.role.is_filled()
        ball_carrier_skill: IBallCarrier = self.BALL_CARRIER.skill

        for seeker_role in seeker_roles:
            if seeker_role.is_filled():
                ...
