import stp
from rj_msgs.msg import RobotIntent

# TODO: mark skill has not been updated, how is it working here (see basicDefense)?
from rj_gameplay.skill import mark


class MarkerRole(stp.role.Role):
    """Role to produce marking behavior"""

    def __init__(self, robot: stp.rc.Robot, face_point, block_point, world_state) -> None:

        self.mark_skill = None

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> RobotIntent:
        if self.mark_skill is None:
            min_robot = None
            min_dist = float('inf')
            for robot in world_state.their_robots:
                dist = np.linalg.norm(robot.pose[:2] - world_state.field.our_goal_loc)
                if dist < min_dist:
                    min_dist = dist
                    min_robot = robot

            self.mark_skill = mark.Mark(
                self.robot, min_robot.pose[:2], world_state.field.our_goal_loc
            )

        intent = self.mark_skill.tick(world_state)

        return intent

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        if self.mark_skill is None:
            return False
        return self.mark_skill.is_done(world_state)

    def __repr__(self):
        return f"MarkerRole(face_point: {self.face_point}, block_point: {self.block_point})"
