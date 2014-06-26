import composite_behavior
import behavior
import skills.move
import constants
import math
import role_assignment
import robocup
import main

class CircleUp(composite_behavior.CompositeBehavior):

    def __init__(self):
        super().__init__(continuous=True)
        self.add_transition(behavior.Behavior.State.start, behavior.Behavior.State.running, lambda: True, 'immediately')
        self.add_transition(behavior.Behavior.State.running, behavior.Behavior.State.completed, lambda: self.all_subbehaviors_completed(), 'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed, behavior.Behavior.State.running, lambda: not self.all_subbehaviors_completed(), "robots aren't lined up")

        # Define circle to circle up on
        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01

        perRobot = (2 * constants.Field.CenterRadius * 1.25) / radius * (180.0 / math.pi)

        ball_pos = main.ball().pos if main.ball() != None else robocup.Point(constants.Field.Width / 2, constants.Field.Length / 2)

        dirvec = (robocup.Point(0,0) - ball_pos).normalized() * radius
        dirvec.rotate(robocup.Point(0,0), -perRobot * 2.5)

        for i in range(6):
            pt = ball_pos + dirvec
            self.add_subbehavior(skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=6 - i)
            dirvec.rotate(robocup.Point(0,0), perRobot)

    def all_subbehaviors_completed(self):
        return False

    def execute_running(self):
        # run subbehaviors
        super().execute_running()

        # set robot attributes
        for b in self.all_subbehaviors():
            if b.robot is not None:
                b.robot.set_avoid_ball_radius(constants.Field.CenterRadius)
                b.robot.face(main.ball().pos)
                b.robot.avoid_all_teammates(True)