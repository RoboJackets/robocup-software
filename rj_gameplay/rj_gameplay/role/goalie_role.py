from typing import Dict, Type, List, Any
import stp

from rj_gameplay.skill import move, receive, pivot_kick  # , line_kick, intercept
import numpy as np

# TODO: settle on unified way to define constants in gameplay
from stp.utils.constants import RobotConstants  # , BallConstants

# import stp.global_parameters as global_parameters
# from stp.local_parameters import Param

from rj_msgs.msg import RobotIntent

# TODO: move to constants file
MIN_WALL_RAD = 0
GOALIE_PCT_TO_BALL = 0.15
DIST_TO_FAST_KICK = 7


def get_goalie_pt(world_state: stp.rc.WorldState) -> np.ndarray:
    """Gives goalie a default location to track the ball from when it is not actively intercepting or capturing the ball.
    :return numpy point
    """
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
    # get in-between ball and goal, staying behind wall
    dist_from_goal = min(GOALIE_PCT_TO_BALL * np.linalg.norm(ball_pt - goal_pt), 1.0)
    mid_pt = goal_pt + (dir_vec * dist_from_goal)
    return mid_pt


"""Calculates point to move to to stop the ball, standing in for an intercept planner.
"""


def get_block_pt(world_state: stp.rc.WorldState, my_pos: np.ndarray) -> np.ndarray:
    pos = world_state.ball.pos
    vel = world_state.ball.vel

    tangent = vel / (np.linalg.norm(vel) + 1e-6)

    # Find out where it would cross
    time_to_cross = np.abs(pos[1] / vel[1]) if np.abs(vel[1]) > 1e-6 else 0
    cross_x = pos[0] + vel[0] * time_to_cross
    cross_position = np.array([np.clip(cross_x, a_min=-0.6, a_max=0.6), 0.0])

    tangent = cross_position - pos
    tangent /= np.linalg.norm(tangent)
    block_pt = np.dot(tangent, my_pos - pos) * tangent + pos

    return block_pt


class GoalieRole(stp.role.Role):
    """Role to produce goalie behavior, which tracks the ball, moves to block if a shot on goal is taken, stays within the goalie box (generally), and clears ball away."""

    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], robot: stp.rc.Robot, brick=False):
        super().__init__(action_client_dict, robot)

        self.brick = brick

        self.move_skill = None
        self.receive_skill = None
        self.pivot_kick_skill = None

    # TODO: there is crusty if-else here, use utility AI or behavior tree or FSM
    #       anything more readable than this (behavior tree prob best fit for current logic)
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        global MIN_WALL_RAD

        # TODO: this calculation is copy-pasted from wall_tactic
        # put into common file (wall calculations?)

        # dist is slightly greater than def_area box bounds
        box_w = world_state.field.def_area_long_dist_m
        box_h = world_state.field.def_area_short_dist_m
        line_w = world_state.field.line_width_m
        # max out of box to cap for goalie
        MAX_OOB = RobotConstants.RADIUS

        if not world_state:
            return None
        if not world_state.ball.visible:
            return None

        ball_speed = np.linalg.norm(world_state.ball.vel)
        ball_pos = world_state.ball.pos
        # ball_dist = np.linalg.norm(world_state.field.our_goal_loc - ball_pos)
        goal_pos = world_state.field.our_goal_loc
        towards_goal = goal_pos - ball_pos

        if self.brick:
            self.move_skill = move.Move(
                robot=self.robot,
                target_point=world_state.field.our_goal_loc,
                face_point=world_state.ball.pos,
            )
            return self.move_skill.tick(world_state)

        if (
            ball_speed < 0.5
            and (
                abs(ball_pos[0]) < box_w / 2 + line_w + MAX_OOB
                and ball_pos[1] < box_h + line_w + MAX_OOB
            )
            and not world_state.game_info.is_stopped()
        ):
            if ball_speed < 1e-6:
                # if ball is stopped and inside goalie box, collect it
                self.receive_skill = receive.Receive(robot=self.robot)
                return self.receive_skill.tick(world_state)
            else:
                # if ball has been stopped already, chip toward center field
                self.pivot_kick_skill = pivot_kick.PivotKick(
                    robot=self.robot, target_point=np.array([0.0, 6.0])
                )
                return self.pivot_kick_skill.tick(world_state)
        else:
            if ball_speed > 0 and np.dot(towards_goal, world_state.ball.vel) > 0.3:
                # if ball is moving and coming at goal, move laterally to block ball
                # TODO (#1676): replace this logic with a real intercept planner
                goalie_pos = (
                    world_state.our_robots[world_state.goalie_id].pose[:2]
                    if world_state.goalie_id is not None
                    else np.array([0.0, 0.0])
                )

                block_point = get_block_pt(world_state, goalie_pos)
                face_point = world_state.ball.pos

                self.move_skill = move.Move(
                    robot=self.robot,
                    target_point=block_point,
                    face_point=face_point,
                )
                return self.move_skill.tick(world_state)

            else:
                # else, track ball normally
                self.move_skill = move.Move(
                    target_point=get_goalie_pt(world_state),
                    face_point=world_state.ball.pos,
                )
                return self.move_skill.tick(world_state)

        if self.pivot_kick_skill is not None and self.pivot_kick_skill.is_done(
            world_state
        ):
            self.pivot_kick_skill = pivot_kick.PivotKick(
                robot=self.robot,
                target_point=np.array([0.0, 6.0]),
                chip=True,
                kick_speed=5.5,
            )

            return self.pivot_kick_skill.tick(world_state)

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # goalie always active
        # TODO: make role end on capture, let passing role take over
        return False
