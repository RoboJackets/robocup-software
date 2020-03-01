import main
import robocup
import behavior
import constants

import standard_play
import skills.line_kick
import tactics.coordinated_pass
import enum
import skills.move
import random
import situational_play_selection

## A basic play for the Defensive restart kick
# Will have two robots move up onto offensive half of field
# will chip to the one directly in front of the kicking robot


class BasicDefensiveKick(standard_play.StandardPlay):

    _situationList = [
        situational_play_selection.SituationalPlaySelector.Situation.DEFENSIVE_KICK
    ] # yapf: disable

    class State(enum.Enum):
        defense_wall = 1
        shot = 2
        scramble = 3

    ##
    # @param num_defenders: number of robots in the wall (default 3)
    # @param curvature: 'curvature' (in radians) of the wall
    # @param mark_point: what point are we defending against (default: ball)
    # @param defender_point: what point are we defending (default: goal)
    # @param defender_spacing: number of robot radii between the centers of the
    #   defenders in the wall
    # @param dist_from_mark: distance from the mark point we want to build the wall
    def __init__(
        self,
        num_defenders=3,
        curvature=.3,
        mark_point=None,
        defender_point=robocup.Point(0, 0),
        defender_spacing=2.5,
        dist_from_mark=.75,
    ):
        super().__init__(continuous=True)

        self.mark_moved = False
        self.active_defenders = num_defenders
        self.number_of_defenders = num_defenders
        self.curvature = 1 * curvature
        self._mark_point = main.ball().pos if mark_point == None else mark_point
        self._defense_point = defender_point
        self.dist_from_mark = dist_from_mark
        self.defender_spacing = defender_spacing

        # Information for movement calculations to reduce redundancy
        self.midpoint = None

        self.add_state(Wall.State.defense_wall,
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Wall.State.defense_wall, lambda: True,
                            "immediately")

    def on_enter_defense_wall(self):
        self.remove_all_subbehaviors()
        self._add_wall_defenders()

    def execute_defense_wall(self):
        if self.active_defenders < self.number_of_defenders:
            #self._update_wall()
            self._add_wall_defenders()
            self.active_defenders = self.number_of_defenders
        elif self.active_defenders > self.number_of_defenders:
            self._remove_wall_defenders()
            self.active_defenders = self.number_of_defenders

    ## moves robot to appropriate positions to form wall
    def _add_wall_defenders(self):
        self.update_midpoint()
        pts = []
        for i in range(self.number_of_defenders):
            pts.append(self.calculate_destination(i))

        # prioritize the middle elements of the wall
        for i in range(len(pts)):
            pt = pts[i]
            self.add_subbehavior(self.WallMove(
                pt, pt if i != 0 and i != len(pts) - 1 else None),
                                 name="robot" + str(i),
                                 required=False)

    ## Remove wall behaviors
    def _remove_wall_defenders(self):
        self.update_midpoint()
        for i in range(len(self.active_defenders)):
            name = "robot" + str(i)
            pt = self.calculate_destination(i)
            if i < self.number_of_defenders:
                self.subbehavior_with_name(name).pos = pt
            else:
                self.remove_subbehavior(name)

    ## Remove all behaviors from wall and rebuild wall
    def _rebuild_wall(self):
        self.remove_all_subbehaviors()
        self._add_wall_defenders()

    ## Finds the point on the arc the defender should move to
    def calculate_destination(self, robot_number):
        defender_number = robot_number - self.number_of_defenders / 2 + .5
        direct = (self.mark_point - self.defense_point).normalized()
        arc_angle = defender_number * self.curvature + math.pi/2
        direct.rotate_origin(arc_angle)
        return self.midpoint - direct * constants.Robot.Radius * self.defender_spacing * defender_number

    ## Update wall's midpoint based on the mark and defense points
    def update_midpoint(self):
        self.midpoint = self.mark_point + (self.defense_point - self.mark_point).normalized() * self.dist_from_mark

    @property
    def defense_point(self):
        return self._defense_point

    # Changes the point we are defending against attack, updates move behaviors
    @defense_point.setter
    def defense_point(self, point):
        self._defense_point = point
        self._update_wall()

    @property
    def num_defenders(self):
        self.number_of_defenders

    @num_defenders.setter
    def num_defenders(self, value):
        self.active_defenders = self.number_of_defenders
        self.number_of_defenders = value

    @property
    def mark_point(self):
        return self._mark_point

    # Changes the point we are defending against, updates move behaviors
    @mark_point.setter
    def mark_point(self, point):
        self._mark_point = point
        self._update_wall()

    ## Recalculate points where the wall defenders should be
    def _update_wall(self):
        self.update_midpoint()
        for i in range(self.number_of_defenders):
            if self.has_subbehavior_with_name("robot" + str(i)):
                behavior = self.subbehavior_with_name("robot" + str(i))
                behavior.pos = self.calculate_destination(i)


    ## Move behavior that prioritizes the center point of the wall
    class WallMove(skills.move.Move):
        def __init__(self, pos=None, center_pt=None):
            super().__init__(pos)
            self.center_pt = center_pt

        def role_requirements(self):
            reqs = super().role_requirements()
            reqs.destination_shape = self.center_pt if self.center_pt else None
            return reqs
