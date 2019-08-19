import single_robot_behavior
import behavior
import robocup
import main
import constants
import role_assignment


class Mark(single_robot_behavior.SingleRobotBehavior):
    '''
    The mark skill will take a robot or mark point and position itself on the line from the ball to the 
    robot or mark_pt
    @param: ratio - this will control how far from the opposing robot the defender will mark 
            (0 is close to ball, 1 is close to mark_pt/robot)
    @param: mark_point - point to mark, this is not required and will overwrite the mark_robot if set
    @param: mark_robot - robot to mark, mark_point is essentially set to mark_robot.pos 
    '''
    def __init__(self):
        super().__init__(continuous=True)
        #Below params are described above @properties
        self._ratio = 0.9
        self._mark_line_thresh = 0.9
        self._mark_robot = None
        self._mark_point = None

        self._target_point = None

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

    def execute_running(self):
        #pylint: disable=no-member
        #Skill does nothing if mark point isn't given AND the ball or robot to mark can't be found
        if self.mark_point is None and \
           (self.mark_robot is None or
            not main.ball().valid or
            not self.mark_robot.visible):
            return

        #Sets the position to mark as the given mark position 
        #or robot position if no mark position is given 
        ball_pos = main.ball().pos
        pos = self.robot.pos
        mark_pos = self.mark_point if self.mark_point is not None else self.mark_robot.pos

        #Finds the line from the ball to the mark position and creates a line between them
        #removing the overlap with the ball on one side and robot on the other
        #This assumes even with mark position parameter that there is a robot there to avoid
        mark_line_dir = (ball_pos - mark_pos).normalized()
        ball_mark_line = robocup.Segment(
            ball_pos - mark_line_dir * constants.Ball.Radius,
            mark_pos + mark_line_dir * 2.0 * constants.Robot.Radius)

        #Drawing for simulator
        main.debug_drawer().draw_line(ball_mark_line, (0, 0, 255), "Mark")

        #Distance from robot to mark line
        mark_line_dist = ball_mark_line.dist_to(pos)

        #Sets target point to nearest point on mark line if the robot is over ball_mark_threshold
        #from the mark line
        # or
        #Sets target point to place on line defined by ratio- 
        #   - 0 being close to ball, 1 close to mark pt
        self._target_point = None
        if mark_line_dist > self.mark_line_thresh:
            self._target_point = ball_mark_line.nearest_point(pos)
        else:
            self._target_point = ball_pos + (
                mark_pos -
                ball_pos).normalized() * self.ratio * ball_mark_line.length()

        #Drawing for simulator
        main.debug_drawer().draw_circle(mark_pos, constants.Robot.Radius * 1.2,
                                        (0, 127, 255), "Mark")

        #Move robot into position and face the ball
        self.robot.move_to(self._target_point)
        #self.robot.face(ball_pos) #Commented out to reduce lateral motion

    #Ratio of distance to mark point v. distance to ball for target point
    @property
    def ratio(self):
        return self._ratio

    @ratio.setter
    def ratio(self, value):
        self._ratio = min(max(value, 0.0), 1.0)

    #Max distance for inclusion of ratio in target point
    #If robot distance exceeds this then it will target the closest point on the mark line
    @property
    def mark_line_thresh(self):
        return self._mark_line_thresh

    @mark_line_thresh.setter
    def mark_line_thresh(self, value):
        self._mark_line_thresh = value

    # Overrides mark_robot with a static point
    @property
    def mark_point(self) -> robocup.Point:
        return self._mark_point

    @mark_point.setter
    def mark_point(self, value: robocup.Point):
        self._mark_point = value

    #Robot to mark (what it sounds like)
    @property
    def mark_robot(self) -> robocup.Robot:
        return self._mark_robot

    @mark_robot.setter
    def mark_robot(self, value: robocup.Robot):
        self._mark_robot = value

    # Choose a robot close to the mark point
    def role_requirements(self):
        req = super().role_requirements()
        if self._target_point is not None:
            req.destination_shape = self._target_point
        return req
