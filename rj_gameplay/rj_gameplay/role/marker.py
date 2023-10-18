import stp
from rj_msgs.msg import RobotIntent

# TODO: mark skill has not been updated, how is it working here (see basicDefense)?
from rj_gameplay.skill import mark


class MarkerRole(stp.role.Role):
    """Role to produce marking behavior"""

    def __init__(self, robot: stp.rc.Robot, target_robot: stp.rc.Robot) -> None:
        super().__init__(robot)

        self.mark_skill = None





















































        self.target_robot = target_robot

    def tick     (
        self, world_state: stp.rc.WorldState, target_robot: stp.rc.Robot
    ) -> RobotIntent:
        if target_robot is not None:
            self.target_robot = target_robot
        if self.mark_skill is None or target_robot is not None:
            self.mark_skill = mark.Mark(
                robot=self.robot, target_robot=self.target_robot
            )

        intent = self.mark_skill.tick(world_state)

        return intent

    def is_done        (           self,world_state: stp.rc.WorldState) -> bool:
        return self._state == "done"
