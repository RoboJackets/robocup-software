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
                 curvature =  0,                            # 'curvature' (in radians) of the wall 
                 mark_point = None,                         # what point we are defending against (default is ball)
                 defender_point = robocup.Point(0, 0),      # what point we are defending (default is goal)
                 defender_spacing = 3.5,                    # number of robot radii between the centers of the defenders in the wall
                 dist_from_mark = 1,                        # distance from the mark point we want to build the wall
                 defender_priorities = [20, 19, 18, 17, 16]): # default defense priorities                       
        super().__init__(continuous=True)

        self.number_of_defenders = num_defenders
        self.curvature = 1 * curvature
        self._mark_point = main.ball().pos if mark_point == None else mark_point
        self._defense_point = defender_point
        self.dist_from_mark = dist_from_mark
        self.defender_spacing = defender_spacing
        self.defender_priorities = defender_priorities

        # Information for movement calculations to reduce redundancy 
        self.midpoint = None

        self.add_state(Wall.State.defense_wall, 
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Wall.State.defense_wall, lambda: True,
                            "immideately")

    def on_enter_defense_wall(self):
        self.remove_all_subbehaviors()
        self.update_midpoint()
        for i, priority in enumerate(self.defender_priorities[:self.number_of_defenders]):
            pt = self.calculate_destination(i)
            self.add_subbehavior(
                skills.move.Move(pt),
                name="robot" + str(i),
                required=False,
                priority=priority)

    # Finds the point on the arc the defender should move to
    def calculate_destination(self, robot_number):
        defender_number = robot_number - self.number_of_defenders / 2 + .5
        direct = (self.mark_point - self.defense_point).normalized()
        arc_angle = defender_number * self.curvature + math.pi/2
        direct.rotate_origin(arc_angle)
        return self.midpoint - direct * constants.Robot.Radius * self.defender_spacing * defender_number

    def update_midpoint(self):
        self.midpoint = self.mark_point + (self.defense_point - self.mark_point).normalized() * self.dist_from_mark

    @property
    def defense_point(self):
        return self._defense_point
    
    # Changes the point we are defending against attack, updates move behaviors
    @defense_point.setter
    def defense_point(self, point):
        self._defense_point = point
        self.update_midpoint()
        for i in range(self.number_of_defenders):
            if self.has_subbehavior_with_name("robot" + str(i)):
                behavior = self.subbehavior_with_name("robot" + str(i))
                behavior.pos = self.calculate_destination(i)

    @property
    def mark_point(self):
        return self._mark_point

    # Changes the point we are defending against, updates move behaviors
    @mark_point.setter
    def mark_point(self, point):
        self._mark_point = point
        self.update_midpoint()
        for i in range(self.number_of_defenders):
            if self.has_subbehavior_with_name("robot" + str(i)):
                behavior = self.subbehavior_with_name("robot" + str(i))
                behavior.pos = self.calculate_destination(i)
