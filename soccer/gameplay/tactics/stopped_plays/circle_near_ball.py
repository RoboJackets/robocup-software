import composite_behavior
import behavior
import skills.move
import constants
import math
import role_assignment
import robocup
import main


## Robots position themselves along a portion of the circle centered at the ball
class CircleNearBall(composite_behavior.CompositeBehavior):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')
        self.add_transition(behavior.Behavior.State.running,
            behavior.Behavior.State.completed,
            lambda: self.all_subbehaviors_completed(),
            'all robots reach target positions')
        self.add_transition(behavior.Behavior.State.completed,
            behavior.Behavior.State.running,
            lambda: not self.all_subbehaviors_completed(),
            "robots aren't lined up")

        # Define circle to circle up on
        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01

        perRobot = (2 * constants.Robot.Radius* 1.25) / radius

        ball_pos = main.ball().pos if main.ball() != None else robocup.Point(constants.Field.Width / 2, constants.Field.Length / 2)

        dirvec = (robocup.Point(0,0) - ball_pos).normalized() * radius

        for i in range(6):
            pt = ball_pos + dirvec
            self.add_subbehavior(skills.move.Move(pt), name="robot" + str(i), required=False, priority=6 - i)
            dirvec.rotate(robocup.Point(0,0), perRobot)


    def all_subbehaviors_completed(self):
        return all([b.behavior_state == behavior.Behavior.State.completed or b.robot == None for b in self.all_subbehaviors()])

    def move_around_circle(self):
        num_robots = 0
        for b in self.all_subbehaviors():
            if b.robot is not None:
                num_robots+=1

        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01
        ball_pos = main.ball().pos if main.ball() != None else robocup.Point(constants.Field.Width / 2, constants.Field.Length / 2)
        circle_ball = robocup.Circle(ball_pos, radius)

        intersection_points = []
        for i in constants.Field.FieldBorders:
            tmp_point = circle_ball.intersects_line(i)
            for j in tmp_point:
                if constants.Field.FieldRect.contains_point(j):
                    intersection_points.append(j)

        angles = []
        candidate_arcs = []
        if len(intersection_points) > 1:
            for i in intersection_points:
                new_angle = (i - circle_ball.center()).angle()
                while new_angle < 0:
                    new_angle = new_angle + math.pi * 2
                while new_angle > math.pi * 2:
                    new_angle = new_angle - math.pi * 2
                angles.append(new_angle)

            counter = 1
            while counter < len(angles):
                candidate_arcs.append(robocup.Arc(circle_ball.center(), radius, angles[counter - 1], angles[counter]))
                counter = counter + 1
            candidate_arcs.append(robocup.Arc(circle_ball.center(), radius, angles[len(angles) - 1], angles[0]))

            i = 0
            while i < len(candidate_arcs):
                angle_between = candidate_arcs[i].end() - candidate_arcs[i].start()
                while angle_between < math.pi * 2:
                    angle_between = angle_between + math.pi * 2
                while angle_between > math.pi * 2:
                    angle_between = angle_between - math.pi * 2
                angle_diff = candidate_arcs[i].start() + (angle_between) / 2.0
                while angle_diff > math.pi * 2:
                    angle_diff = angle_diff - math.pi * 2
                midpoint = (candidate_arcs[i].center() + robocup.Point(radius, 0))
                midpoint.rotate(candidate_arcs[i].center(), angle_diff)
                if not constants.Field.FieldRect.contains_point(midpoint):
                    candidate_arcs.pop(i)
                else:
                    i = i + 1

            candidate_arcs.sort(key=lambda arc: arc.end() - arc.start())
            # TODO guard for none
            final_arc = candidate_arcs[0]
        else:
            midpoint = (circle_ball.center() + robocup.Point(radius, 0))
            if not constants.Field.FieldRect.contains_point(midpoint):
                final_arc = robocup.Arc(robocup.Point(constants.Field.Width / 4.0, constants.Field.Length / 4.0), radius, math.pi / 2, 5 * math.pi / 2)
            else:
                final_arc = robocup.Arc(circle_ball.center(), radius, math.pi / 2, 5 * math.pi / 2)




        arc_angle = final_arc.end() - final_arc.start()
        while arc_angle < 0:
            arc_angle = arc_angle + math.pi * 2
        perRobot = arc_angle / (num_robots + 1)

        dirvec = robocup.Point(radius, 0)
        dirvec.rotate(robocup.Point(0,0), final_arc.start())
        dirvec.rotate(robocup.Point(0,0), perRobot)

        for i in range(6):
            pt = final_arc.center() + dirvec
            self.subbehavior_with_name("robot" + str(i)).pos = pt
            dirvec.rotate(robocup.Point(0,0), perRobot)

        # set robot attributes
        for b in self.all_subbehaviors():
            if b.robot is not None:
                b.robot.set_avoid_ball_radius(constants.Field.CenterRadius)
                b.robot.face(main.ball().pos)
                b.robot.avoid_all_teammates(True)

    def execute_completed(self):
        self.move_around_circle()


    def execute_running(self):
        self.move_around_circle()
