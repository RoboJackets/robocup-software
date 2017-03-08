import composite_behavior
import behavior
import skills.move
import constants
import math
import role_assignment
import robocup
import main


## Robots position themselves along a portion of the circle centered at the ball
class CircleOnCenter(composite_behavior.CompositeBehavior):
    def __init__(self, min_robots=0):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')
        self.add_transition(behavior.Behavior.State.running,
                            behavior.Behavior.State.completed,
                            lambda: self.all_subbehaviors_completed(),
                            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
                            behavior.Behavior.State.running,
                            lambda: not self.all_subbehaviors_completed(),
                            "robots aren't lined up")

        self.min_robots = min_robots

        # Define circle to circle up on
        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01

        perRobot = (2 * constants.Robot.Radius * 1.25) / radius

        ball_pos = robocup.Point(0, constants.Field.Length / 2)

        dirvec = (robocup.Point(0, 0) - ball_pos).normalized() * radius

        for i in range(6):
            req = i < min_robots
            pt = ball_pos + dirvec if min_robots > 0 else robocup.Point(
                -constants.Field.Width, -constants.Field.Length)
            self.add_subbehavior(
                skills.move.Move(pt),
                name="robot" + str(i),
                required=req,
                priority=6 - i)
            dirvec.rotate(robocup.Point(0, 0), perRobot)

    def goto_center(self):
        num_robots = 0
        for b in self.all_subbehaviors():
            if b.robot is not None:
                num_robots += 1

        num_robots = max(self.min_robots, num_robots)

        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01

        perRobot = math.pi / max(num_robots, 1)

        ball_pos = robocup.Point(0, constants.Field.Length / 2)

        dirvec = (robocup.Point(0, 0) - ball_pos).normalized() * radius
        dirvec.rotate(robocup.Point(0, 0), -perRobot * ((num_robots - 1) / 2))

        for i in range(6):
            pt = ball_pos + dirvec if num_robots > 0 else robocup.Point(
                -constants.Field.Width, -constants.Field.Length)
            if num_robots < 1:
                pt = robocup.Point(-5, -5)
            self.subbehavior_with_name("robot" + str(i)).pos = pt
            dirvec.rotate(robocup.Point(0, 0), perRobot)

        # set robot attributes
        for b in self.all_subbehaviors():
            if b.robot is not None:
                b.robot.set_avoid_ball_radius(constants.Field.CenterRadius)
                b.robot.face(main.ball().pos)

    def execute_completed(self):
        self.goto_center()

    def execute_running(self):
        self.goto_center()
