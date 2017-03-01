import standard_play
import behavior
import tactics.positions.defender
import tactics.stopped.circle_on_center
import skills.mark
import main
import robocup
import constants
import planning_priority


class TheirKickoff(standard_play.StandardPlay):

    # Distance from the center line we should mark from (the mark target distance from the line)
    LineBuffer = constants.Robot.Radius * 3

    # Distance from center to mark if no robot is found
    DefaultDist = constants.Field.CenterDiameter

    # Ratio of the field to consider robots as a threat
    FieldRatio = 3.0 / 4

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        # Add a center blocker
        self.add_subbehavior(
            tactics.stopped.circle_on_center.CircleOnCenter(
                # TODO find a way to do this without hard coding 3 defense/goalie robots (or make those constants)
                min_robots=1 if (main.our_robots() is not None) and len(
                    main.our_robots()) > 3 else 0),
            'circle_up',
            priority=15,
            required=True)

        # Add two marker robots (with lower than defense priority)
        mark_one = skills.mark.Mark()
        self.add_subbehavior(mark_one,
                             'mark_one',
                             priority=planning_priority.PIVOT_KICK + 1,
                             required=False)
        mark_two = skills.mark.Mark()
        self.add_subbehavior(mark_two,
                             'mark_two',
                             priority=planning_priority.PIVOT_KICK,
                             required=False)

    def absmin(self, value, floor_val):
        if value <= 0:
            return min(value, -floor_val)
        else:
            return max(value, floor_val)

    def execute_running(self):
        their_robots = main.their_robots()
        mark_one = self.subbehavior_with_name('mark_one')
        mark_two = self.subbehavior_with_name('mark_two')

        centerCircle = robocup.Circle(constants.Field.CenterPoint,
                                      constants.Field.CenterRadius)

        # Don't select robots that are
        # 1. Not on our side of the field
        # 2. behind or inside the goal circle
        mark_robot_right = list(filter(
            lambda robot: robot.pos.x >= 0 and robot.pos.y < constants.Field.Length * TheirKickoff.FieldRatio and constants.Field.FieldRect.contains_point(robot.pos) and not centerCircle.contains_point(robot.pos),
            their_robots))

        # Don't select robots that are
        # 1. Not on our side of the field
        # 2. behind or inside the goal circle
        # 3. Not the robot selected before
        mark_robot_left = list(filter(
            lambda robot: robot.pos.x <= 0 and robot.pos.y < constants.Field.Length * TheirKickoff.FieldRatio and constants.Field.FieldRect.contains_point(robot.pos) and not centerCircle.contains_point(robot.pos) and robot != mark_one.mark_robot,
            their_robots))

        # Special cases
        if len(mark_robot_left) + len(mark_robot_right) == 0:
            # Can't do anything
            mark_robot_left = None
            mark_robot_right = None
        elif len(mark_robot_left) + len(mark_robot_right) == 1:
            if len(mark_robot_left) == 1:
                mark_robot_right = mark_robot_left[0]
                mark_robot_left = None
            else:
                mark_robot_right = mark_robot_right[0]
                mark_robot_left = None
        elif len(mark_robot_left) == 0:
            mark_robot_right = mark_robot_right
            mark_robot_left = mark_robot_right
        elif len(mark_robot_right) == 0:
            mark_robot_right = mark_robot_left
            mark_robot_left = mark_robot_left
        # Else, everything can proceed as normal (pick best one from each side)

        # Make every element a list to normalize for the next step
        if type(mark_robot_right) is not list and mark_robot_right is not None:
            mark_robot_right = [mark_robot_right]
        if type(mark_robot_left) is not list and mark_robot_left is not None:
            mark_robot_left = [mark_robot_left]

        # Select best robot from candidate lists
        selected = None
        if mark_robot_right is not None:
            mark_robot_right = min(mark_robot_right,
                                   key=lambda robot: robot.pos.y).pos
            selected = robocup.Point(mark_robot_right)
        else:
            mark_robot_right = robocup.Point(TheirKickoff.DefaultDist,
                                             constants.Field.Length / 2)
        # Set x and y seperately as we want a constant y value (just behind the kick off line)
        mark_robot_right.y = min(
            constants.Field.Length / 2 - TheirKickoff.LineBuffer,
            mark_robot_right.y)
        mark_robot_right.x = self.absmin(mark_robot_right.x,
                                         TheirKickoff.DefaultDist)
        mark_one.mark_point = mark_robot_right

        # Do the same thing as above on the left robot.
        if mark_robot_left is not None:
            # Don't mark the same robot twice
            mark_robot_left = filter(
                lambda x: True if selected is None else not x.pos.nearly_equals(selected),
                mark_robot_left)
            mark_robot_left = min(mark_robot_left,
                                  key=lambda robot: robot.pos.y).pos
        else:
            mark_robot_left = robocup.Point(-TheirKickoff.DefaultDist,
                                            constants.Field.Length / 2)
        mark_robot_left.y = min(
            constants.Field.Length / 2 - TheirKickoff.LineBuffer,
            mark_robot_left.y)
        mark_robot_left.x = self.absmin(mark_robot_left.x,
                                        TheirKickoff.DefaultDist)
        mark_two.mark_point = mark_robot_left

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_setup_state() and gs.is_their_kickoff() else float(
            "inf")

    @classmethod
    def is_restart(cls):
        return True
