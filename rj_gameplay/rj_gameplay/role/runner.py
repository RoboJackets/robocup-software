import stp.rc
import stp.role
from rj_msgs.msg import RobotIntent

from rj_gameplay.skill import move


class RunnerRole(stp.role.Role):

    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)
        self.curMove = 0
        self.move_skill = None

    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:

        moves = [
            world_state.field.their_right_corner,
            world_state.field.their_left_corner,
            world_state.field.our_left_corner,
            world_state.field.our_right_corner,
        ]
        
        if self.is_done(world_state):
            self.curMove = (self.curMove + 1) % 4

        self.move_skill = move.Move(
            robot=self.robot,
            target_point=moves[self.curMove],
            face_point=moves[self.curMove]
        )


        intent = self.move_skill.tick(world_state)
        return intent

    def is_done(self, world_state) -> bool:
        if self.move_skill is not None:
            return self.move_skill.is_done(world_state)
