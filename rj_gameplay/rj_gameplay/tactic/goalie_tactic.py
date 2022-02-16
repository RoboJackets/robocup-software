import stp

<<<<<<< HEAD
from dataclasses import dataclass
from typing import Dict, Generic, List, Optional, Tuple, Type, TypeVar, Any
=======
from rj_gameplay.role import goalie_role
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

from rj_msgs.msg import RobotIntent

from typing import List, Tuple


class GoalieTactic(stp.tactic.Tactic):
    """Wrapper for the Goalie Role that handles assigning said role to whichever Robot is our goalie."""

    def __init__(self, world_state: stp.rc.WorldState, goalie_id: int):
        """Special case where we want only robot 0 to be goalie, from init."""
        super().__init__(world_state)

<<<<<<< HEAD
class GoalieCost(role.CostFn):
    def __call__(
        self,
        robot: rc.Robot,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:
        if world_state.game_info is not None:
            if robot.id == world_state.goalie_id:
                return 0.0

        return 10000000

    def unassigned_cost_fn(
        self,
        prev_result: Optional["RoleResult"],
        world_state: rc.WorldState,
    ) -> float:

        # TODO: Implement real unassigned cost function
        return role.BIG_STUPID_NUMBER_CONST_FOR_UNASSIGNED_COST_PLS_CHANGE


def get_goalie_pt(world_state: rc.WorldState) -> np.ndarray:
    """Finds point for goalie to best be in to block a shot.
    :return numpy point
    """
    # TODO: param server any constant from stp/utils/constants.py (this includes BallConstants)
    ball_pt = world_state.ball.pos
    goal_pt = world_state.field.our_goal_loc

    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
    # get in-between ball and goal, staying behind wall
    dist_from_goal = min(GOALIE_PCT_TO_BALL * np.linalg.norm(ball_pt - goal_pt), 1.0)
    mid_pt = goal_pt + (dir_vec * dist_from_goal)
    return mid_pt


def get_block_pt(world_state: rc.WorldState, my_pos: np.ndarray) -> np.ndarray:
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


class GoalieTactic(tactic.ITactic):
    def __init__(self, action_client_dict: Dict[Type[Any], List[Any]], brick=False):

        self._action_client_dict = action_client_dict

        self.brick = brick

        # init skills

        # TODO: pass robot through plays to here?/ maybe None is ok
        self.move_se = tactic.SkillEntry(
            move.Move(action_client_dict, None, ignore_ball=True)
        )
        self.receive_se = tactic.SkillEntry(receive.Receive(action_client_dict, None))
        self.pivot_kick_se = tactic.SkillEntry(
            line_kick.LineKickSkill(
                action_client_dict,
                None,
                target_point=np.array([0.0, 6.0]),
                chip=True,
                kick_speed=5.5,
            )
        )

        # TODO: rename cost_list to role_cost in other gameplay files
        self.role_cost = GoalieCost()

    def compute_props(self):
        pass

    def create_request(self, **kwargs) -> role.RoleRequest:
        """Creates a sane default RoleRequest.
        :return: A list of size 1 of a sane default RoleRequest.
        """
        pass

    def get_requests(
        self, world_state: rc.WorldState, props
    ) -> List[tactic.RoleRequests]:
        global MIN_WALL_RAD
        """
        :return: A list of role requests for move skills needed
        """

        # TODO: this calculation is copy-pasted from wall_tactic
        # put into common param file: https://www.geeksforgeeks.org/global-keyword-in-python/

        # dist is slightly greater than def_area box bounds
        box_w = world_state.field.def_area_long_dist_m
        box_h = world_state.field.def_area_short_dist_m
        line_w = world_state.field.line_width_m
        # max out of box to cap for goalie
        MAX_OOB = RobotConstants.RADIUS

        role_requests = {}
        if world_state and world_state.ball.visible:
            ball_speed = np.linalg.norm(world_state.ball.vel)
            ball_pos = world_state.ball.pos
            ball_dist = np.linalg.norm(world_state.field.our_goal_loc - ball_pos)
            goal_pos = world_state.field.our_goal_loc
            towards_goal = goal_pos - ball_pos

            if self.brick:
                self.move_se.skill.target_point = world_state.field.our_goal_loc
                self.move_se.skill.face_point = world_state.ball.pos
                role_requests[self.move_se] = [
                    role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                ]
                return role_requests

            if (
                ball_speed < 0.5
                and (
                    abs(ball_pos[0]) < box_w / 2 + line_w + MAX_OOB
                    and ball_pos[1] < box_h + line_w + MAX_OOB
                )
                and not world_state.game_info.is_stopped()
            ):
                self.move_se = tactic.SkillEntry(
                    move.Move(
                        action_client_dict=self._action_client_dict, ignore_ball=True
                    )
                )
                if ball_speed < 1e-6:
                    # if ball is stopped and inside goalie box, collect it
                    role_requests[self.receive_se] = [
                        role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                    ]
                else:
                    # if ball has been stopped already, chip toward center field
                    self.pivot_kick_se.skill.target_point = np.array([0.0, 6.0])
                    role_requests[self.pivot_kick_se] = [
                        role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                    ]
            else:
                if ball_speed > 0 and np.dot(towards_goal, world_state.ball.vel) > 0.3:
                    # if ball is moving and coming at goal, move laterally to block ball
                    # TODO (#1676): replace this logic with a real intercept planner
                    goalie_pos = (
                        world_state.our_robots[world_state.goalie_id].pose[:2]
                        if world_state.goalie_id is not None
                        else np.array([0.0, 0.0])
                    )
                    self.move_se.skill.target_point = get_block_pt(
                        world_state, goalie_pos
                    )
                    self.move_se.skill.face_point = world_state.ball.pos
                    role_requests[self.move_se] = [
                        role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                    ]
                else:
                    # else, track ball normally
                    self.move_se.skill.target_point = get_goalie_pt(world_state)
                    self.move_se.skill.face_point = world_state.ball.pos
                    role_requests[self.move_se] = [
                        role.RoleRequest(role.Priority.HIGH, True, self.role_cost)
                    ]
        if self.pivot_kick_se.skill.is_done(world_state):
            self.pivot_kick_se = tactic.SkillEntry(
                line_kick.LineKickSkill(
                    self._action_client_dict,
                    None,
                    target_point=np.array([0.0, 6.0]),
                    chip=True,
                    kick_speed=5.5,
                )
            )

        return role_requests
=======
        # TODO: rather than passing in hardcoded goalie id on init, gameplay node should
        #       have a set robot and pass that (in case robot 0 is ejected or broken)
        self._role_requests.append(
            (stp.role.cost.PickRobotById(goalie_id), goalie_role.GoalieRole)
        )

    def init_roles(self, world_state: stp.rc.WorldState) -> None:
        # only has one role, but it's easier to copy-paste the structure
        for i, robot in enumerate(self.assigned_robots):
            role = self._role_requests[i][1]
            if role is goalie_role.GoalieRole:
                self.assigned_roles.append(role(robot))
>>>>>>> bce13ce53ddb2ecb9696266d980722c34617dc15

    def tick(
        self, world_state: stp.rc.WorldState
    ) -> List[Tuple[int, RobotIntent]]:  # (id, intent)

        # assumes all roles requested are filled, because tactic is one unit
        if len(self.assigned_roles) != len(self._role_requests):
            self.init_roles(world_state)

        # only has one role request, but it's easier to copy-paste the structure
        robot_intents = []
        for i in range(len(self.assigned_roles)):
            role = self.assigned_roles[i]
            robot_intents.append((role.robot.id, role.tick(world_state)))
        return robot_intents

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # special case: we know the only role is Goalie, so we borrow that is_done()
        return self.assigned_roles[0].is_done(world_state)
