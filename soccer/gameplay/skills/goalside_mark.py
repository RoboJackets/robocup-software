import single_robot_behavior
import behavior
import robocup
import main
import constants
import role_assignment


class Goalside_Mark(single_robot_behavior.SingleRobotBehavior):
    '''
    The goalside mark will take a robot or mark point and position itself on line with the best shot from 
    the position of the robot/mark_point.
    @param: ratio - this will control how far from the opposing robot the defender will mark (0 is close to robot)
    '''
    #pylint: disable=no-member

    def __init__(self):
        super().__init__(continuous=True)
        #Below params are described above @properties
        self._ratio = 0.2
        #This threshold is lower than mark to ensure we get goal side of the opponent fast, 
        #then back off based on ratio

        self._mark_line_thresh = constants.Robot.Radius*4
        self._mark_robot = None
        self._mark_point = None

        self._target_point = None

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")
        self.kick_eval = robocup.KickEvaluator(main.system_state())
        self.mark_pos = None
        self.adjusted_mark_pos = None

    def execute_running(self):
        #Skill does nothing if mark point isn't given AND the ball or robot to mark can't be found
        if self.robot is None or (self.mark_point is None and \
           (self.mark_robot is None or
            not main.ball().valid or
            not self.mark_robot.visible)):
            return

        #Finds the line from the mark position to the shot point and creates a line between them
        #removing the overlap with the ball on one side and robot on the other
        #This assumes even with mark position parameter that there is a robot there to avoid
        self._reset_mark_pos()
        mark_line, shot_pt = self.get_mark_segment()

        #Drawing for simulator 
        main.debug_drawer().draw_line(mark_line, (0, 0, 255), "Mark")

        #Distance from robot to mark line
        mark_line_dist = mark_line.dist_to(self.robot.pos)

        #Sets target point to nearest point on mark line if the robot is over ball_mark_threshold
        #from the mark line
        # or
        #Sets target point to place on line defined by ratio- 
        #   - 0 being close to ball, 1 close to mark pt
        self._target_point = None
        if mark_line_dist > self.mark_line_thresh:
            self._target_point = mark_line.nearest_point(self.robot.pos)
        else:
            self._target_point = self.adjusted_mark_pos - (
                self.mark_pos -
                shot_pt).normalized() * self.ratio * mark_line.length()

        #Drawing for simulator
        main.debug_drawer().draw_circle(self.mark_pos, constants.Robot.Radius *
                                        1.2, (0, 127, 255), "Mark")

        #Move robot into position and face the ball
        self.robot.move_to(self._target_point)
        self.robot.face(main.ball().pos)

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

    # Sets the position to mark as the given mark position 
    # or robot position if no mark position is given
    def _reset_mark_pos(self):
        self.mark_pos = self.mark_point if self.mark_point is not None else self._mark_robot.pos

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

    def get_mark_segment(self):
        # Finds the line segment between the ball and the highest danger shot point
        # Cuts off the portion of the line that is inside of the goal box
        # @return: LineSegment to defend on, shot_point
        # This will also set 1 intermediate value to self:
        # self.adjusted_mark_pos - the offset mark pos depending on if we defend a spot (assumed to be the ball) or a robot
        #                        - this defines the closest point to the mark_pos our robot will go to defend

        #Define the segments where the defender can go closest the goal

        offset = constants.Robot.Radius
        goal_rect_padded = constants.Field.OurGoalZoneShapePadded(offset)

        #Find best shot point from threat
        self.kick_eval.add_excluded_robot(self.robot) #FIX: Should we really exclude all of our robots but the goalie?
        shot_pt, shot_score = self.kick_eval.eval_pt_to_our_goal(self.mark_pos)
        self.kick_eval.excluded_robots.clear()

        #End the mark line segment 1 radius away from the opposing robot
        #Or 1 ball radius away if marking a position
        if self.mark_point is None:
            self.adjusted_mark_pos = self.mark_pos - (self.mark_pos - shot_pt).normalized() * 2 * constants.Robot.Radius
        else:
            self.adjusted_mark_pos = self.mark_pos - (self.mark_pos - shot_pt).normalized() * constants.Ball.Radius


        shot_seg = robocup.Segment(self.adjusted_mark_pos , shot_pt)
        tmp = goal_rect_padded.segment_intersection(shot_seg)
        if tmp is None:
            return None, shot_pt

        intersections = sorted(tmp, key=lambda pt: pt.y, reverse=True)
        return robocup.Segment(self.adjusted_mark_pos, intersections[0]), shot_pt
