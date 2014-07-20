import single_robot_behavior
import behavior
import robocup
import main

# wraps up OurRobot.move() into a Skill so we can use it in the play system more easily
class Intercept(single_robot_behavior.SingleRobotBehavior):

    def __init__(self, pos=None):
        super().__init__(continuous=True)

        # self.ball_line = lambda: robocup.Segment(robocup.Point(0,0), robocup.Point(1,1))
        self.ball_line = lambda: robocup.Segment(main.ball().pos, main.ball().pos + (main.ball().vel.normalized() * 8.))

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True, 'immediately')


    def execute_running(self):
        if self.robot != None and main.ball().valid:
            self.target_pos = self.ball_line().nearest_point(self.robot.pos)
            self.robot.move_to(self.target_pos)
            self.robot.face(main.ball().pos)

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.destination_shape = self.ball_line()
        return reqs
