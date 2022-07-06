import stp.rc
import stp.role
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import move


class Runner(stp.role.Role):
    """
    In many ways, this role behaves similar to the DumbMove Role. It will move to the target point while facing the point. This would act as a wrapper to the move skill.

    :param robot: Robot that will be performing this role
    :type robot: stp.rc.Robot
    :param target_point: the target point that the robot will attempt to move to
    :type target_point: (, optional)
    """

    def __init__(self, world_state: stp.rc.WorldState, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.curr = 0
        self.target_points = [
            world_state.field.our_left_corner,
            world_state.field.our_right_corner,
            world_state.field.their_right_corner,
            world_state.field.their_left_corner,
        ]
        self.target_point = self.target_points[0]
        self.face_point = self.target_point

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

        if (
            self.move_skill is not None
            and self.move_skill.is_done(world_state)
            and self.target_points
        ):
            skill_needs_update = True
            if self.curr < len(self.target_points) - 1:
                self.target_point = self.target_points[self.curr + 1]
                self.curr += 1
            else:
                self.target_point = self.target_points[0]
                self.curr = 0
            self.move_skill = None

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
