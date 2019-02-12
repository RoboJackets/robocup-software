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
        self.num_robots = 0

        self.add_circle_subbehaviors()

    def add_circle_subbehaviors(self):
        #create move behaviors with no position (we can't assign position because we don't know how many bots we have)
        self.remove_all_subbehaviors()
        for i in range(6):
            req = i < self.min_robots
            self.add_subbehavior(skills.move.Move(),
                                 name="robot" + str(i),
                                 required=req,
                                 priority=6 - i)

    def goto_center(self):
        num_robots = 0
        for b in self.all_subbehaviors():
            if b.robot is not None:
                num_robots += 1

        #if the number of robots has changed, recreate move behaviors to match new number of robots
        if (self.num_robots != num_robots):
            self.num_robots = num_robots
            self.add_circle_subbehaviors()

        num_robots = max(self.min_robots, num_robots)

        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01

        perRobot = math.pi / max(num_robots, 1)

        ball_pos = robocup.Point(0, constants.Field.Length / 2)

        dirvec = (robocup.Point(0, 0) - ball_pos).normalized() * radius
        dirvec.rotate(robocup.Point(0, 0), -perRobot * ((num_robots - 1) / 2))

        #assign points to the behaviors with robots
        for i in range(num_robots):
            pt = ball_pos + dirvec
            self.subbehavior_with_name("robot" + str(i)).pos = pt
            dirvec.rotate(robocup.Point(0, 0), perRobot)

        # set robot attributes
        for b in self.all_subbehaviors():
            if b.robot is not None:
                b.robot.set_avoid_ball_radius(constants.Field.CenterRadius)
                b.robot.face(main.ball().pos)

    def execute_running(self):
        self.goto_center()
