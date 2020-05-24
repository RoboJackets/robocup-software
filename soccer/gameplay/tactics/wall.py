from typing import Optional
import composite_behavior
import role_assignment
import evaluation
import behavior
import constants
import robocup
import main
import enum
import math
import skills.move


## This tactic builds a wall a certain distance from point A, blocking point B.
class Wall(composite_behavior.CompositeBehavior):
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
        num_defenders: int = 3,
        curvature: float = .3,
        mark_point: Optional[robocup.Point] = None,
        defender_point: robocup.Point = robocup.Point(0, 0),
        defender_spacing: float = 2.5,
        dist_from_mark: float = .75,
    ):
        super().__init__(continuous=True)
        self.mark_moved = False
        self.active_defenders = num_defenders
        self.number_of_defenders = num_defenders
        self.curvature = 1 * curvature
        self._mark_point = main.ball(
        ).pos if mark_point is None else mark_point
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

    def on_enter_defense_wall(self) -> None:
        self.remove_all_subbehaviors()
        self._add_wall_defenders()

    def execute_defense_wall(self) -> None:
        if self.active_defenders < self.number_of_defenders:
            # self._update_wall()
            self._add_wall_defenders()
            self.active_defenders = self.number_of_defenders
        elif self.active_defenders > self.number_of_defenders:
            self._remove_wall_defenders()
            self.active_defenders = self.number_of_defenders

    ## moves robot to appropriate positions to form wall
    def _add_wall_defenders(self) -> None:
        self.update_midpoint()
        pts = []
        for i in range(self.number_of_defenders):
            pts.append(self.calculate_destination(i))

        # prioritize the middle elements of the wall
        for i in range(len(pts)):
            pt = pts[i]
            subbhvr_name = f"robot{i}"
            # Oswin: I'm not sure why we have this logic here...
            center_pt = pt if i != 0 and i != len(pts) - 1 else None

            # We might have 0 < active_defenders < number_of_defenders
            # so we need to check that we're not adding a duplicate
            if self.has_subbehavior_with_name(subbhvr_name):
                move_subbhvr = self.subbehavior_with_name(subbhvr_name)
                assert isinstance(move_subbhvr, self.WallMove)

                # Update pos and center_pt instead of removing and adding
                # the subbehavior so it doesn't cause swapping of roles
                move_subbhvr.pos = pt
                move_subbhvr.center_pt = center_pt
                continue

            self.add_subbehavior(self.WallMove(pt, center_pt),
                                 name=subbhvr_name,
                                 required=False)

    ## Remove wall behaviors
    def _remove_wall_defenders(self) -> None:
        self.update_midpoint()
        for i in range(self.active_defenders):
            name = "robot" + str(i)
            pt = self.calculate_destination(i)
            if i < self.number_of_defenders:
                move_subbhvr = self.subbehavior_with_name(name)
                assert isinstance(move_subbhvr, self.WallMove)
                move_subbhvr.pos = pt
            else:
                self.remove_subbehavior(name)

    ## Remove all behaviors from wall and rebuild wall
    def _rebuild_wall(self) -> None:
        self.remove_all_subbehaviors()
        self._add_wall_defenders()

    ## Finds the point on the arc the defender should move to
    def calculate_destination(self, robot_number: int) -> robocup.Point:
        defender_number = robot_number - self.number_of_defenders / 2 + .5
        direct: robocup.Point = (self.mark_point -
                                 self.defense_point).normalized()
        arc_angle = defender_number * self.curvature + math.pi / 2
        direct.rotate_origin(arc_angle)
        return self.midpoint - direct * constants.Robot.Radius * self.defender_spacing * defender_number

    ## Update wall's midpoint based on the mark and defense points
    def update_midpoint(self) -> None:
        self.midpoint = self.mark_point + (self.defense_point - self.mark_point
                                           ).normalized() * self.dist_from_mark

    @property
    def defense_point(self) -> robocup.Point:
        return self._defense_point

    # Changes the point we are defending against attack, updates move behaviors
    @defense_point.setter
    def defense_point(self, point: robocup.Point) -> None:
        self._defense_point = point
        self._update_wall()

    @property
    def num_defenders(self) -> int:
        return self.number_of_defenders

    @num_defenders.setter
    def num_defenders(self, value: int) -> None:
        self.active_defenders = self.number_of_defenders
        self.number_of_defenders = value

    @property
    def mark_point(self) -> robocup.Point:
        return self._mark_point

    # Changes the point we are defending against, updates move behaviors
    @mark_point.setter
    def mark_point(self, point: robocup.Point):
        self._mark_point = point
        self._update_wall()

    ## Recalculate points where the wall defenders should be
    def _update_wall(self) -> None:
        self.update_midpoint()
        for i in range(self.number_of_defenders):
            if self.has_subbehavior_with_name("robot" + str(i)):
                move_subbhvr = self.subbehavior_with_name("robot" + str(i))
                assert isinstance(move_subbhvr, self.WallMove)
                move_subbhvr.pos = self.calculate_destination(i)

    ## Move behavior that prioritizes the center point of the wall
    class WallMove(skills.move.Move):
        def __init__(self,
                     pos: Optional[robocup.Point] = None,
                     center_pt: Optional[robocup.Point] = None):
            super().__init__(pos)
            self.center_pt = center_pt

        def role_requirements(  # type: ignore
                self) -> role_assignment.RoleRequirements:
            reqs = super().role_requirements()
            reqs.destination_shape = self.center_pt if self.center_pt else None
            return reqs
