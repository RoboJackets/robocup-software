import composite_behavior
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

    def __init__(
            self,
            num_defenders=3,  # number of defenders we're making the wall with (default 3)
            curvature=.3,  # 'curvature' (in radians) of the wall 
            mark_point=None,  # what point we are defending against (default is ball)
            defender_point=robocup.Point(
                0, 0),  # what point we are defending (default is goal)
            defender_spacing=2.5,  # number of robot radii between the centers of the defenders in the wall
            dist_from_mark=.75,  # distance from the mark point we want to build the wall
            defender_priorities=[20, 19, 18, 17, 16],
            contest_ball=False
    ):  # default defense priorities                       
        super().__init__(continuous=True)

        is_ball_free = lambda: main.ball().vel.mag() < 1 and min([(main.ball(
        ).pos - rob.pos).mag() for rob in main.system_state(
        ).their_robots]) > min([(main.ball().pos - rob.pos).mag()
                                for rob in main.system_state().our_robots])

        self.mark_moved = False
        self.active_defenders = num_defenders
        self.number_of_defenders = num_defenders
        self.curvature = 1 * curvature
        self._mark_point = main.ball().pos if mark_point == None else mark_point
        self._defense_point = defender_point
        self.dist_from_mark = dist_from_mark
        self.defender_spacing = defender_spacing
        self.defender_priorities = defender_priorities
        self.contest_ball = contest_ball

        # Information for movement calculations to reduce redundancy
        self.midpoint = None

        self.add_state(Wall.State.defense_wall,
                       behavior.Behavior.State.running)
        self.add_state(Wall.State.shot, behavior.Behavior.State.running)
        self.add_state(Wall.State.scramble, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            Wall.State.defense_wall, lambda: True,
                            "immideately")
        self.add_transition(Wall.State.defense_wall,
                            Wall.State.shot, lambda: False, "on shot")
        self.add_transition(
            Wall.State.defense_wall,
            Wall.State.scramble, lambda: evaluation.ball.we_are_closer(
            ) and evaluation.ball.moving_slow(), "ball free")
        self.add_transition(
            Wall.State.scramble,
            Wall.State.defense_wall, lambda: not evaluation.ball.we_are_closer(
            ) or not evaluation.ball.moving_slow(), "ball captured")

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

    def on_enter_scramble(self):
        self._remove_wall_defenders()
        if self.contest_ball:
            self.number_of_defenders = self.number_of_defenders - 1
            self.add_subbehavior(
                skills.pivot_kick.
                PivotKick(),  # TODO figure out what to do in scramble
                name="robotCapture")

    def on_exit_scramble(self):
        self._add_wall_defenders()
        if self.contest_ball:
            self.number_of_defenders = self.number_of_defenders + 1
            self.remove_subbehavior("robotCapture")

    def execute_scramble(self):
        pass  #print('scrambling')

    def execute_defense_wall(self):
        if self.active_defenders < self.number_of_defenders:
            #self._update_wall()
            self._add_wall_defenders()
            self.active_defenders = self.number_of_defenders
        elif self.active_defenders > self.number_of_defenders:
            self._remove_wall_defenders()
            self.active_defenders = self.number_of_defenders

    ## Returns true if some team has possession of the ball
    def is_ball_not_free(self):
        return main.ball().vel.mag() > 1 or min([(main.ball().pos - rob.pos).mag() \
            for rob in main.system_state().their_robots]) <= min([(main.ball().pos - rob.pos).mag() \
                for rob in main.system_state().our_robots])

    ## Returns true if the ball was shot
    def is_ball_shot(self):
        SHOT_THRESH = 2
        return main.ball().vel.mag() > SHOT_THRESH and (
            main.ball().vel).normalized().dot(main.ball().vel) > .9

    ## moves robot to appropriate positions to form wall
    def _add_wall_defenders(self):
        self.update_midpoint()
        for i, priority in enumerate(
                self.defender_priorities[:self.number_of_defenders]):
            name = "robot" + str(i)
            pt = self.calculate_destination(i)
            if i < self.active_defenders:
                self.subbehavior_with_name(name).pos = pt
            else:
                self.add_subbehavior(skills.move.Move(pt), name=name)

    ## Remove wall behaviors
    def _remove_wall_defenders(self):
        self.update_midpoint()
        for i, priority in enumerate(
                self.defender_priorities[:self.active_defenders]):
            name = "robot" + str(i)
            pt = self.calculate_destination(i)
            if i < self.number_of_defenders:
                self.subbehavior_with_name(name).pos = pt
            else:
                self.remove_subbehavior(name)

    ## Remove all behaviors from wall and rebuild wall
    def _rebuild_wall(self):
        self.remove_all_subbehaviors()
        self.update_midpoint()
        for i, priority in enumerate(
                self.defender_priorities[:self.active_defenders]):
            name = "robot" + str(i)
            pt = self.calculate_destination(i)
            self.add_subbehavior(skills.move.Move(pt), name=name)

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
