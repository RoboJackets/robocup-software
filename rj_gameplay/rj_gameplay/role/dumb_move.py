from typing import Dict, Type, List, Any
import stp.role
import stp.rc

from rj_gameplay.skill import move

from rj_msgs.msg import RobotIntent


class DumbMove(stp.role.Role):
    """Wrapper for the Move Skill. Named "Dumb" because it does not have much other functionality (breaking the ideal of a "complex" single-robot Role)."""

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], robot: stp.rc.Robot, target_point, face_point) -> None:
        super().__init__(action_client_dict, robot)

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
                action_client_dict=self.action_client_dict,
                robot=self.robot,
                target_point=self.target_point,
                face_point=self.face_point,
            )

        intent = self.move_skill.tick(world_state)
        return intent

    def is_done(self, world_state) -> bool:
        return self.move_skill.is_done(world_state)
