import composite_behavior
import behavior
import constants
import robocup
import main
import enum
import math
import skills.move

# This tactic builds a wall a certain distance from point A, blocking point B.
class Wall(composite_behavior.CompositeBehavior):
    class State(enum.Enum):
        defense_wall = 1

    def __init__(self,
                 num_defenders = 3,                         # number of defenders we're making the wall with (default 3)
                 curvature = 0,                             # curvature (in radians) of the wall 0 - pi/2
                 mark_point = None,                         # what point we are defending against (default is ball)
                 defender_point = robocup.Point(0, 0),      # what point we are defending (default is goal)
                 defender_spacing = 3,                      # number of robot radii between the centers of the defenders in the wall
                 dist_from_mark = 1):                       # distance from the mark point we want to build the wall
        super().__init__(continuous=True)

        self.number_of_defenders = num_defenders
        self.curvature = curvature
        self._mark_point = main.ball().pos if mark_point == None else mark_point
        self._defense_point = defender_point
        self.dist_from_mark = dist_from_mark
        self.defender_spacing = defender_spacing

        self.add_state(Wall.State.defense_wall, 
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Wall.State.defense_wall, lambda: True,
                            "immediately")

        self.add_transition(
            Wall.State.defense_wall,
            behavior.Behavior.State.completed, lambda: self.completed(), "robot kicked")

    def on_enter_defense_wall(self):
        self.remove_all_subbehaviors()
        midpoint = self.arc_midpoint()
        #angle = (midpoint - self.defense_point).angle()
        angle = (math.atan2(midpoint.x - self.defense_point.x, midpoint.y - self.defense_point.y))
        for i in range(self.number_of_defenders):
            pt = None
            if self.number_of_defenders % 2 != 0:
                pt = self.point_on_arc(midpoint, angle, i - int(self.number_of_defenders / 2))
            else:
                pt = self.point_on_arc(midpoint, angle, i - self.number_of_defenders / 2 + .5)
            self.add_subbehavior(
                skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=self.number_of_defenders - i)

    def completed(self):
        for bhvr in self.all_subbehaviors():
             if not bhvr.is_done_running():
                return False
        self.remove_all_subbehaviors()
        return True

    # Finds the point on the wall (arc) at which the defender robot should move to
    def point_on_arc(self, midpoint, angle, defender_number):
        x_pt = midpoint.x - defender_number * (constants.Robot.Radius * self.defender_spacing) * math.cos(angle + (defender_number * self.curvature))
        y_pt = midpoint.y + defender_number * (constants.Robot.Radius * self.defender_spacing) * math.sin(angle + (defender_number * self.curvature))
        return robocup.Point(x_pt,y_pt)

    # Finds the point on the line between the defense and mark point that is the distance specified from the mark point
    def arc_midpoint(self):
        return self.mark_point + (self.defense_point - self.mark_point).normalized() * self.dist_from_mark

    @property
    def defense_point(self):
        return self._defense_point

    @property
    def mark_point(self):
        return self._mark_point

    @defense_point.setter
    def defense_point(self, point):
        self.remove_all_subbehaviors()
        self._defense_point = point
        self.on_enter_defense_wall()

    @mark_point.setter
    def mark_point(self, point):
        self.remove_all_subbehaviors()
        self._mark_point = point
        self.on_enter_defense_wall()
