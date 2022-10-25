import numpy as np
import stp
from rj_msgs.msg import RobotIntent

# TODO: settle on unified way to define constants in gameplay
from stp.utils.constants import RobotConstants  # , BallConstants

from rj_gameplay.skill import line_kick, move, pivot_kick, receive

# import stp.global_parameters as global_parameters
# from stp.local_parameters import Param


# TODO: move to constants file
MIN_WALL_RAD = 0
GOALIE_PCT_TO_BALL = 0.15
DIST_TO_FAST_KICK = 7


#def get_goalie_pt(world_state: stp.rc.WorldState) -> np.ndarray:
#    """Gives goalie a default location to track the ball from when it is not actively intercepting or capturing the ball.
#    :return numpy point
#    """
#    ball_pt = world_state.ball.pos
#    goal_pt = world_state.field.our_goal_loc

#    dir_vec = (ball_pt - goal_pt) / np.linalg.norm(ball_pt - goal_pt)
#    # get in-between ball and goal, staying behind wall
#    dist_from_goal = min(GOALIE_PCT_TO_BALL * np.linalg.norm(ball_pt - goal_pt), 1.0)
#    mid_pt = goal_pt + (dir_vec * dist_from_goal)
#    return mid_pt


#"""Calculates point to move to to stop the ball, standing in for an intercept planner.
#"""


#def get_block_pt(world_state: stp.rc.WorldState, my_pos: np.ndarray) -> np.ndarray:
#    pos = world_state.ball.pos
#    vel = world_state.ball.vel

#    tangent = vel / (np.linalg.norm(vel) + 1e-6)

#    # Find out where it would cross
#    time_to_cross = np.abs(pos[1] / vel[1]) if np.abs(vel[1]) > 1e-6 else 0
#    cross_x = pos[0] + vel[0] * time_to_cross
#    cross_position = np.array([np.clip(cross_x, a_min=-0.6, a_max=0.6), 0.0])

#    tangent = cross_position - pos
#    tangent /= np.linalg.norm(tangent)
#    block_pt = np.dot(tangent, my_pos - pos) * tangent + pos

#    return block_pt

def findVel(world_state: stp.rc.WorldState, robo_pos, targetPoint: np.ndarray) -> np.ndarray:
    #print(robo_pos[0])
    #print(robo_pos[1])
    
    roboCoords = np.ndarray([2], dtype = float)
    roboCoords =[robo_pos[0], robo_pos[1]]
    
    vel = np.array([2], dtype=float)
    vel = [(targetPoint[0] - roboCoords[0]), (targetPoint[1] - roboCoords[1])]
    
    return vel
    


class RunnerRole(stp.role.Role):
    """Role to produce runner behavior, which makes an arbitrary robot run around the perimeter of the field."""

    def __init__(self, robot: stp.rc.Robot, brick=False):
        super().__init__(robot)

        self.brick = brick

        self.move_skill = None
        self.receive_skill = None
        self.pivot_kick_skill = None
        self.cornerNum = 2;

    # TODO: there is crusty if-else here, use utility AI or behavior tree or FSM
    #       anything more readable than this (behavior tree prob best fit for current logic)
#    def tick(self, world_state: stp.rc.WorldState, corner: np.ndarray, robo_id: int) -> RobotIntent:
    def tick(self, world_state: stp.rc.WorldState) -> RobotIntent:
        global MIN_WALL_RAD

        # TODO: this calculation is copy-pasted from wall_tactic
        # put into common file (wall calculations?)

#        # dist is slightly greater than def_area box bounds
#        box_w = world_state.field.def_area_long_dist_m
#        box_h = world_state.field.def_area_short_dist_m
#        line_w = world_state.field.line_width_m
#        # max out of box to cap for goalie
#        MAX_OOB = RobotConstants.RADIUS

        if not world_state:
            return None
        if not world_state.ball.visible:
            return None

#        ball_speed = np.linalg.norm(world_state.ball.vel) + 1e-6
#        ball_pos = world_state.ball.pos
#        # ball_dist = np.linalg.norm(world_state.field.our_goal_loc - ball_pos)
#        goal_pos = world_state.field.our_goal_loc
#        towards_goal = goal_pos - ball_pos
        
        
        ###############################################################################
        
        ## use wrapper class and rc.py to send controls here --> repeat this function 4 times
        ## so that it goes to all 4 corners
        
#        corners = []
#        corners.append(world_state.field.our_left_corner)
#        corners.append(world_state.field.our_right_corner)
#        corners.append(world_state.field.their_right_corner)
#        corners.append(world_state.field.their_left_corner)
        
#        for i in range(len(corners)):
        
        runner_pos = (
            world_state.our_robots[self.robot.id].pose[:2]
            if world_state.goalie_id is not None
            else np.array([0.0, 0.0])
            )
        
#        vel = np.ndarray([2], dtype=float)
#        vel = findVel(world_state, runner_pos, corner)
#        print(vel)
        
        corners = []
        corners.append(world_state.field.our_left_corner)
        corners.append(world_state.field.our_right_corner)
        corners.append(world_state.field.their_right_corner)
        corners.append(world_state.field.their_left_corner)
        
        for i in range(0, len(corners)):
            print(self.robot.pose[:2])
            print(world_state.our_robots[self.robot.id].pose[:2])
            #print("Working")
            #print(self.robot.pose[0])
            #print((corners[self.cornerNum])[0])
            #print(self.robot.pose[1])
            #print((corners[self.cornerNum])[1])
            
            if ((abs(runner_pos[0] - (corners[self.cornerNum])[0]) < 0.1) and (abs(runner_pos[1] - (corners[self.cornerNum])[1]) < 0.1)):
                print("working")
                if (self.cornerNum == 3):
                    vel = findVel(world_state, runner_pos, corners[0])
                    self.move_skill = move.Move(
                        robot=self.robot,
                        target_point=corners[0],
                        target_vel=vel
#                        face_point=face_point,
                        )
                    self.cornerNum = 0
                    
                else:
                    vel = findVel(world_state, runner_pos, corners[self.cornerNum+1])
                    self.move_skill = move.Move(
                        robot=self.robot,
                        target_point=corners[self.cornerNum + 1],
                        target_vel=vel
#                        face_point=face_point,
                        )
                    self.cornerNum = self.cornerNum + 1
                    
                return self.move_skill.tick(world_state)
        
        vel = findVel(world_state, runner_pos, corners[self.cornerNum])    
        self.move_skill = move.Move(
            robot=self.robot,
            target_point=corners[self.cornerNum],
            target_vel=vel
#            face_point=face_point,
            )
        return self.move_skill.tick(world_state)
        
        ###############################################################################
        
        
#        if self.brick:
#            self.move_skill = move.Move(
#                robot=self.robot,
#                target_point=world_state.field.our_goal_loc,
#                face_point=world_state.ball.pos,
#            )
#            return self.move_skill.tick(world_state)

#        if (
#            ball_speed < 0.5
#            and (
#                abs(ball_pos[0]) < box_w / 2 + line_w + MAX_OOB
#                and ball_pos[1] < box_h + line_w + MAX_OOB
##            )
#            and not world_state.game_info.is_stopped()
#        ):
#            if ball_speed < 1e-6:
#                # if ball is stopped and inside goalie box, collect it
#                self.receive_skill = receive.Receive(robot=self.robot)
#                return self.receive_skill.tick(world_state)
#            else:
#                # if ball has been stopped already, chip toward center field
#                self.pivot_kick_skill = line_kick.LineKick(
#                    robot=self.robot, target_point=np.array([0.0, 6.0])
#                )
#                return self.pivot_kick_skill.tick(world_state)
#        else:
#            if ball_speed > 0 and np.dot(towards_goal, world_state.ball.vel) > 0.3:
#                # if ball is moving and coming at goal, move laterally to block ball
#                # TODO (#1676): replace this logic with a real intercept planner
#                goalie_pos = (
#                    world_state.our_robots[world_state.goalie_id].pose[:2]
#                    if world_state.goalie_id is not None
#                    else np.array([0.0, 0.0])
#                )

#                block_point = get_block_pt(world_state, goalie_pos)
#                face_point = world_state.ball.pos

#                self.move_skill = move.Move(
#                    robot=self.robot,
#                    target_point=block_point,
#                    face_point=face_point,
#                )
#                return self.move_skill.tick(world_state)

#            else:
                # else, track ball normally
#                self.move_skill = move.Move(
#                    target_point=get_goalie_pt(world_state),
#                    face_point=world_state.ball.pos,
#                )
#                return self.move_skill.tick(world_state)

#        if self.pivot_kick_skill is not None and self.pivot_kick_skill.is_done(
#            world_state
#        ):
#            self.pivot_kick_skill = pivot_kick.PivotKick(
#                robot=self.robot,
#                target_point=np.array([0.0, 6.0]),
#                chip=True,
#                kick_speed=5.5,
#            )

#            return self.pivot_kick_skill.tick(world_state)

    def is_done(self, world_state: stp.rc.WorldState) -> bool:
        # goalie always active
        # TODO: make role end on capture, let passing role take over
        return False
