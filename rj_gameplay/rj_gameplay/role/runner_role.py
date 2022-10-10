import stp.rc
import stp.role
from rj_gameplay.skill import move
import numpy as np
from rj_msgs.msg import RobotIntent

class RunnerRole(stp.role.Role):
    current_target = 0
    target_points = [np.array([-3.0,9.0]), np.array([-3.0,0.0]), np.array([3.0,0.0]), np.array([3.0,9.0])]
    ticks_since_last_check = 0

    def __init__(self, robot: stp.rc.Robot) -> None:
        super().__init__(robot)

    
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        x_coord = self.robot.pose[0]
        y_coord = self.robot.pose[1]
        target_point = self.target_points[self.current_target]

        if (self.ticks_since_last_check > 600):
            if True:#abs(x_coord - target_point[0]) < 0.5 and abs(y_coord - target_point[1]) < 0.5:
                self.current_target = (self.current_target + 1) % 4

            target_point = self.target_points[self.current_target]
            self.ticks_since_last_check = 0
        
        self.ticks_since_last_check += 1
        move_skill = move.Move(robot=self.robot, target_point=target_point)
        intent = move_skill.tick(world_state)
        return intent

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        return False
