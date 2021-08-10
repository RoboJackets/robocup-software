# import RobotIntent msg
# import params
# import typing
# import math, np, etc.

class Move():
    """Sends a move command to motion planning. Implements behavior tree."""
    def __init__(self, robot_id: int):
        self.robot_id = robot_id

    def tick(self, 
            world_state: rc.WorldState, 
            target_point: np.ndarray, 
            target_vel: np.ndarray = np.array([0.0, 0.0]), 
            face_angle : Optional[float] = None, 
            face_point : Optional[np.ndarray] = None) -> msg.RobotIntent:
        # TODO: make this a behavior tree

        intent = msg.RobotIntent()
        
        path_command = PathTargetMotionCommand()
        path_command.target.position = Point(x=self.target_point[0],y=self.target_point[1])
        path_command.target.velocity = Point(x=self.target_vel[0],y=self.target_vel[1])

        if(self.face_angle is not None):
            path_command.override_angle=[self.face_angle]

        if(self.face_point is not None):
            path_command.override_face_point=[Point(x=self.face_point[0], y=self.face_point[1])]
        
        if self.is_def_restart:
            ball_circle = geo_msg.ShapeSet()
            ctr = geo_msg.Point(x=self.ball_pos[0], y=self.ball_pos[1])
            # rad = RobotConstants.RADIUS * 4
            rad = 1.5 # m
            ball_circle.circles = [geo_msg.Circle(center=ctr, radius=rad)]
            intent.local_obstacles = ball_circle

        intent.motion_command.path_target_command = [path_command]   
        intent.is_active = True
        return intent

    def is_done(self) -> bool:
        # action server call
        pass
