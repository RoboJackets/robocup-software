import stp.rc
import stp.role
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import move


class DumbMove(stp.role.Role):
    """Wrapper for the Move Skill. Named "Dumb" because it does not have much other functionality (breaking the ideal of a "complex" single-robot Role)."""

    def __init__(self, robot: stp.rc.Robot, target_point, face_point) -> None:
        super().__init__(robot)

        self.target_point = target_point
        self.face_point = face_point

        self.move_skill = None

    def tick(
        self, world_state: stp.rc.WorldState, target_point=None, face_point=None
    ) -> RobotIntent:

        skill_needs_update = self.move_skill is None

        if target_point is not None:
            self.target_point = target_point
            skill_needs_update = True

        if face_point is not None:
            self.face_point = face_point
            skill_needs_update = True

        # create skill with correct target & face_point
        if skill_needs_update:
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=self.target_point,
                face_point=self.face_point,
            )

        intent = self.move_skill.tick(world_state)
        return intent

    def is_done(self, world_state) -> bool:
        if self.move_skill is not None:
            return self.move_skill.is_done(world_state)
