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

    BackupBallLocation = robocup.Point(0, constants.Field.Length / 4.0)

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

        i = 0
        for pt in self.get_circle_points(6):
            self.add_subbehavior(skills.move.Move(pt), name="robot" + str(i), required=False, priority=6 - i)
            i = i + 1


    def all_subbehaviors_completed(self):
        return all([b.behavior_state == behavior.Behavior.State.completed or b.robot == None for b in self.all_subbehaviors()])

    def get_circle_points(self, num_of_points):
        radius = constants.Field.CenterRadius + constants.Robot.Radius + 0.01
        ball_pos = main.ball().pos if main.ball() != None else robocup.Point(constants.Field.Width / 2, constants.Field.Length / 2)
        circle_ball = robocup.Circle(ball_pos, radius)

        intersection_points = []
        for i in constants.Field.FieldBorders:

            for j in circle_ball.intersects_line(i):
                # Using near_point because of rounding errors
                if constants.Field.FieldRect.near_point(j, 0.001):
                    intersection_points.append(j)

        angles = []
        candidate_arcs = []
        if len(intersection_points) > 1:
            for i in intersection_points:
                new_angle = (i - circle_ball.center).angle()
                new_angle = self.normalize_angle(new_angle)
                angles.append(new_angle)

            # Get angles going sequencially
            angles.sort()

            counter = 1
            while counter < len(angles):
                candidate_arcs.append(robocup.Arc(circle_ball.center, radius, angles[counter - 1], angles[counter]))
                counter = counter + 1
            candidate_arcs.append(robocup.Arc(circle_ball.center, radius, angles[len(angles) - 1], angles[0]))

            i = 0
            while i < len(candidate_arcs):
                angle_between = candidate_arcs[i].end() - candidate_arcs[i].start()
                angle_between = self.normalize_angle(angle_between)

                angle_diff = candidate_arcs[i].start() + (angle_between) / 2.0
                angle_diff = self.normalize_angle(angle_diff)

                midpoint = (candidate_arcs[i].center() + robocup.Point(radius, 0))
                midpoint.rotate(candidate_arcs[i].center(), angle_diff)
                if not constants.Field.FieldRect.contains_point(midpoint):
                    candidate_arcs.pop(i)
                else:
                    i = i + 1

            candidate_arcs.sort(key=lambda arc: self.normalize_angle(arc.end() - arc.start()), reverse=True)

            if len(candidate_arcs) <= 0:
                final_arc = robocup.Arc(CircleNearBall.BackupBallLocation, radius, math.pi / 2, 5 * math.pi / 2)
            else:
                final_arc = candidate_arcs[0]
        else:
            midpoint = (circle_ball.center + robocup.Point(radius, 0))
            if not constants.Field.FieldRect.contains_point(midpoint):
                final_arc = robocup.Arc(CircleNearBall.BackupBallLocation, radius, math.pi / 2, 5 * math.pi / 2)
            else:
                final_arc = robocup.Arc(circle_ball.center, radius, math.pi / 2, 5 * math.pi / 2)

        arc_angle = final_arc.end() - final_arc.start()
        arc_angle = self.normalize_angle(arc_angle)

        perRobot = arc_angle / (num_of_points + 1)

        dirvec = robocup.Point(radius, 0)
        dirvec.rotate(robocup.Point(0,0), final_arc.start())
        dirvec.rotate(robocup.Point(0,0), perRobot)

        final_points = []
        for i in range(num_of_points):
            pt = final_arc.center() + dirvec
            final_points.append(pt)
            dirvec.rotate(robocup.Point(0,0), perRobot)

        return final_points

    def execute_completed(self):
        num_robots = 0
        for b in self.all_subbehaviors():
            if b.robot is not None:
                num_robots+=1

        i = 0
        for pt in self.get_circle_points(num_robots):
            self.subbehavior_with_name("robot" + str(i)).pos = pt
            i = i + 1

        # set robot attributes
        for b in self.all_subbehaviors():
            if b.robot is not None:
                b.robot.set_avoid_ball_radius(constants.Field.CenterRadius)
                b.robot.face(main.ball().pos)
                b.robot.avoid_all_teammates(True)



    # Makes an angle > 0, < pi * 2
    def normalize_angle(self, angle):
        # TODO make this O(1) and move to cpp
        while angle > math.pi * 2:
            angle = angle - math.pi * 2
        while angle < 0:
            angle = angle + math.pi * 2
        return angle

    def execute_running(self):
        num_robots = 0
        for b in self.all_subbehaviors():
            if b.robot is not None:
                num_robots+=1

        i = 0
        for pt in self.get_circle_points(num_robots):
            self.subbehavior_with_name("robot" + str(i)).pos = pt
            i = i + 1

        # set robot attributes
        for b in self.all_subbehaviors():
            if b.robot is not None:
                b.robot.set_avoid_ball_radius(constants.Field.CenterRadius)
                b.robot.face(main.ball().pos)
                b.robot.avoid_all_teammates(True)
