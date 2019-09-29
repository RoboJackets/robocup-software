import single_robot_composite_behavior
import skills._kick
import behavior
import skills.aim
import skills.capture
import role_assignment
import robocup
import constants
import main
import math
import planning_priority
import evaluation.ball
from enum import Enum


# PivotKick drives up to the ball and captures it, then aims at a specified target and kicks/chips
# Note: PivotKick recalculates aim_target_point from the target at every iteration
class PivotKick(single_robot_composite_behavior.SingleRobotCompositeBehavior,
                skills._kick._Kick):
    class State(Enum):
        capturing = 1
        aiming = 2
        aimed = 3
        kicking = 4

    def __init__(self):
        single_robot_composite_behavior.SingleRobotCompositeBehavior.__init__(
            self)
        skills._kick._Kick.__init__(self)

        for state in PivotKick.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            PivotKick.State.capturing, lambda: True,
                            'immediately')
        self.add_transition(
            PivotKick.State.capturing, PivotKick.State.aiming,
            lambda: self.subbehavior_with_name('capture').state == behavior.Behavior.State.completed,
            'has ball')

        self.add_transition(
            PivotKick.State.aiming, PivotKick.State.aimed,
            lambda: self.subbehavior_with_name('aim').state == skills.aim.Aim.State.aimed,
            'aim error < threshold')

        self.add_transition(
            PivotKick.State.aimed, PivotKick.State.aiming,
            lambda: self.subbehavior_with_name('aim').state == skills.aim.Aim.State.aiming and not self.enable_kick,
            'aim error > threshold')

        self.add_transition(
            PivotKick.State.aiming,
            PivotKick.State.kicking, lambda: self.opp_robot_blocking(),
            'opp robot blocking')

        self.add_transition(
            PivotKick.State.aimed, PivotKick.State.kicking, lambda: self.
            enable_kick or self.facing_opp_goal() or self.opp_robot_blocking(),
            'kick enabled')

        self.add_transition(PivotKick.State.kicking,
                            behavior.Behavior.State.completed,
                            lambda: self.robot.just_kicked(), 'kick complete')

        self.add_transition(
            PivotKick.State.aiming, PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed,
            'fumble')
        self.add_transition(
            PivotKick.State.aimed, PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed,
            'fumble')
        self.add_transition(
            PivotKick.State.kicking, PivotKick.State.capturing,
            lambda: self.subbehavior_with_name('aim').state == behavior.Behavior.State.failed and not self.robot.just_kicked(),
            'fumble')

        # default parameters
        self.dribbler_power = constants.Robot.Dribbler.StandardPower
        self.aim_params = {'desperate_timeout': float("inf")}

    def facing_opp_goal(self):
        robot = self.subbehavior_with_name('aim').robot

        if robot is None:
            return False

        # L is left post
        # R is right post
        # T is target aiming point
        # U is us
        # L      T      R
        #  \     |     /
        #   \    |    /
        #    \   |   /
        #     \  |  /
        #       \|/
        #        U
        # Angle LUT + Angle RUT should equal Angle LUR if vector UT is between vectors UL and UR
        #
        #
        # L      R      T
        #  \     |     /
        #   \    |    /
        #    \   |   /
        #     \  |  /
        #       \|/
        #        U
        # Here, Angle LUT + Angle RUT is much larger than Angle LUR since vector UT is outside vectors UL and UR

        left_goal_post  = robocup.Point(-constants.Field.GoalWidth / 2, constants.Field.Length)
        right_goal_post = robocup.Point(constants.Field.GoalWidth / 2, constants.Field.Length)

        bot_to_left_goal_post = left_goal_post - robot.pos
        bot_to_right_goal_post = right_goal_post - robot.pos
        bot_forward_vector = robot.pos + robocup.Point(math.cos(robot.angle), math.sin(robot.angle))

        angle_left_goal_post_diff  = bot_forward_vector.angle_between( bot_to_left_goal_post )
        angle_right_goal_post_diff = bot_forward_vector.angle_between( bot_to_right_goal_post )
        angle_goal_post_diff       = bot_to_left_goal_post.angle_between( bot_to_right_goal_post )

        # Add a small amount for any errors in these math functions
        small_angle_offset = 0.01

        # We are aiming at the goal
        if (angle_left_goal_post_diff + angle_right_goal_post_diff + small_angle_offset <= angle_goal_post_diff):
            print('EARLY KIck')
            main.debug_drawer().draw_text('Early kick', robot.pos, 'PivotKick')
            return True

        return False

    def opp_robot_blocking(self):
        if (self.robot is None):
            return False

        # Closest opp robot in any direction
        # To us, not the ball   
        closest_opp_robot = None
        closest_opp_dist = float("inf")
        for r in main.their_robots():
            if ((r.pos - self.robot.pos).mag() < closest_opp_dist):
                closest_opp_robot = r
                closest_opp_dist = (r.pos - self.robot.pos).mag()

        # Only do this if a robot is in range
        robot_in_range = closest_opp_dist < .2 + 2 * constants.Robot.Radius

        aim_dir = robocup.Point.direction(self.robot.angle)
        robot_dir = (closest_opp_robot.pos - self.robot.pos)

        # Only trigger if they are infront of us
        robot_in_front = aim_dir.dot(robot_dir) > 0

        closest_pt = robocup.Line(
            self.robot.pos,
            self.robot.pos + aim_dir).nearest_point(closest_opp_robot.pos)

        does_hit_robot = (closest_opp_robot.pos - closest_pt
                          ).mag() < constants.Robot.Radius

        facing_their_side = robocup.Point.direction(self.robot.angle).y > 0

        ret = (facing_their_side and robot_in_range and robot_in_front and
               does_hit_robot)

        if ret:
            print("Panic kick")
            main.debug_drawer().draw_text('panic kick', self.robot.pos,
                                          (255, 255, 255), 'PivotKick')

        return ret


    # The speed to drive the dribbler at during aiming
    # If high, adds lift to kick
    # Default: full power
    # FIXME: defaulting to full power probably isn't the best - the C++ version sais full power in the header then actually used 50.  maybe use half speed?
    @property
    def dribbler_power(self):
        return self._dribbler_power

    @dribbler_power.setter
    def dribbler_power(self, value):
        self._dribbler_power = int(value)

    # if you want to set custom error thresholds, etc to be used during aiming,
    # set this to a dictionary with the appropriate keys and values
    # default: {'desperate_timeout': float("inf")}
    @property
    def aim_params(self):
        return self._aim_params

    @aim_params.setter
    def aim_params(self, value):
        self._aim_params = value

    # The point near the target point that we're currently aimed at, whether we want to be or not
    # If we kicked right now, the ball would pass through this point
    def current_shot_point(self):
        if self.has_subbehavior_with_name('aim'):
            return self.subbehavior_with_name('aim').current_shot_point()
        else:
            return None

    def is_steady(self):
        if self.has_subbehavior_with_name('aim'):
            return self.subbehavior_with_name('aim').is_steady()
        else:
            return None

    def remove_aim_behavior(self):
        if self.has_subbehavior_with_name('aim'):
            self.remove_subbehavior('aim')

    def on_enter_capturing(self):
        # We disable the shot obstacle in this step just in case we had a
        # failure and we're restarting this play
        self.enable_shot_obstacle = False

        self.remove_aim_behavior()
        self.robot.unkick()
        capture = skills.capture.Capture()
        capture.dribbler_power = self.dribbler_power
        self.add_subbehavior(capture, 'capture', required=True)
        # FIXME: tell capture to approach from a certain direction so we're already lined up?
    def on_exit_capturing(self):
        self.remove_subbehavior('capture')

    def set_aim_params(self):
        aim = self.subbehavior_with_name('aim')
        aim.target_point = self.aim_target_point
        aim.dribbler_power = self.dribbler_power
        for key, value in self.aim_params.items():
            setattr(aim, key, value)

    def execute_running(self):
        self.recalculate_aim_target_point()
        super().execute_running()
        self.robot.set_planning_priority(planning_priority.PIVOT_KICK)

    def on_enter_aiming(self):
        self.enable_shot_obstacle = True
        if not self.has_subbehavior_with_name('aim'):
            aim = skills.aim.Aim()
            self.add_subbehavior(aim, 'aim', required=True)
            self.set_aim_params()

    def execute_aiming(self):
        self.set_aim_params()

        if isinstance(self.target, robocup.Segment):
            for i in range(2):
                main.debug_drawer().draw_line(
                    robocup.Line(main.ball().pos, self.target.get_pt(i)),
                    constants.Colors.Blue, "PivotKick")

    def on_exit_aiming(self):
        # we don't remove the 'aim' subbehavior here because if we're going to the
        # kicking state, we want to keep it around
        pass

    def execute_kicking(self):
        self.set_aim_params()
        vel = (self.aim_target_point - self.robot.pos).normalized(0.5)
        self.robot.set_world_vel(vel)
        if self.use_chipper and self.robot.has_chipper():
            self.robot.chip(self.chip_power)
        else:
            self.robot.kick(self.kick_power)
        if (evaluation.ball.robot_has_ball(self.robot)):
            self.robot.kick_immediately()

    def on_exit_running(self):
        self.remove_aim_behavior()

    def role_requirements(self):
        reqs = super().role_requirements()

        for r in role_assignment.iterate_role_requirements_tree_leaves(reqs):
            # try to be near the ball
            if main.ball().valid:
                r.destination_shape = main.ball().pos
            if self.use_chipper:
                r.chipper_preference_weight = role_assignment.PreferChipper
            r.require_kicking = True

            # Increase the importance of Position to make robots closer to the ball clear it during defense
            r.position_cost_multiplier = 1.5

        return reqs
