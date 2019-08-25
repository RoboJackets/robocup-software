import single_robot_behavior
import behavior
import robocup
import main


# wraps up OurRobot.move() into a Skill so we can use it in the play system more easily
class Intercept(single_robot_behavior.SingleRobotBehavior):
    def __init__(self, pos=None, faceBall=True):
        super().__init__(continuous=True)

        self.faceBall = faceBall

        self._shape_constraint = None

        self.ball_line = lambda: robocup.Segment(main.ball(
        ).pos, main.ball().pos + (main.ball().vel.normalized() * 8.))

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

    def execute_running(self):
        #pylint: disable=no-member
        if self.robot is not None and main.ball().valid:
            if self.shape_constraint is None:
                self.target_pos = self.ball_line().nearest_point(
                    self.robot.pos)
            elif isinstance(self.shape_constraint, robocup.Segment):
                self.target_pos = self.shape_constraint.segment_intersection(
                    self.ball_line())
                if self.target_pos is None:
                    self.target_pos = self.ball_line().nearest_point(
                        self.robot.pos)
                    self.target_pos = self.shape_constraint.nearest_point(
                        self.target_pos)
                main.debug_drawer().draw_line(self.shape_constraint,
                                              (0, 255, 0), "Debug")
            else:
                self.target_pos = self.ball_line().nearest_point(
                    self.robot.pos)


            # Intercept works better at high accelerations and speeds
            # but due to lag with bad motion control, it doesn't actually
            # end up moving robots in the way
            # TODO(motion-control): Swap back to intercept
            #self.robot.intercept(self.target_pos)
            self.robot.move_to(self.target_pos)

            if self.faceBall:
                self.robot.face(main.ball().pos)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.destination_shape = self.ball_line()
        return reqs

    """
    @brief If not None, the intercepting robot will remain on this shape.
    """

    @property
    def shape_constraint(self):
        return self._shape_constraint

    @shape_constraint.setter
    def shape_constraint(self, value):
        self._shape_constraint = value
