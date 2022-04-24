import stp
from rj_msgs.msg import RobotIntent

# TODO: mark skill has not been updated, how is it working here (see basicDefense)?
from rj_gameplay.skill import mark


class MarkerRole(stp.role.Role):
    """Role to produce marking behavior"""

    def __init__(self, robot: stp.rc.Robot, face_point, block_point, world_state) -> None:
        """
        face/block point of format:
        {
            "ball", None
        }
        where key is type of point, and val is specifier if needed

        Dict should only have 1 item in it

        options are "ball", "goal", "robot", "pt"
        where robot requires an ID and pt requires an np.ndarray
        """

        super().__init__(robot)

        # convert both Dicts to actual points
        self.face_point = world_state.ball.pos # default
        if "ball" in face_point:
            pass # done by default
        elif "goal" in face_point:
            self.face_point = world_state.field.our_goal_loc
        elif "robot" in face_point:
            robot_id = face_point["robot"]
            their_robot = world_state.their_robots[robot_id]
            if not their_robot.visible:
                print("Mark skill asked to mark invisible robot; defaulting to face ball")
            self.face_point = their_robot.pose[0:2]
        elif "pt" in face_point:
            self.face_point = face_point["pt"]
        else:
            print("Invalid face_point given to mark role, defaulting to face ball")

        # convert both Dicts to actual points
        self.block_point = world_state.field.our_goal_loc # default
        if "ball" in block_point:
            self.block_point = world_state.ball.pos
        elif "goal" in block_point:
            pass # done by default
        elif "robot" in block_point:
            robot_id = block_point["robot"]
            their_robot = world_state.their_robots[robot_id]
            if not their_robot.visible:
                print("Mark skill asked to mark invisible robot; defaulting to block goal")
            self.block_point = their_robot.pose[0:2]
        elif "pt" in block_point:
            self.block_point = block_point["pt"]
        else:
            print("Invalid block_point given to mark role, defaulting to block goal")

        print("marker role conversion")
        print(self.face_point)
        print(self.block_point)

        self.mark_skill = None

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> RobotIntent:
        if self.mark_skill is None:
            self.mark_skill = mark.Mark(
                self.robot, self.face_point, self.block_point
            )

        intent = self.mark_skill.tick(world_state)

        return intent

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        if self.mark_skill is None:
            return False
        return self.mark_skill.is_done(world_state)

    def __repr__(self):
        return f"MarkerRole(face_point: {self.face_point}, block_point: {self.block_point})"
