# import RobotIntent msg
# import params
# import math, numpy, etc
# import typing
from skill import move, kick, receive

class Goalie(ITactic):
    """
    Goalie blocks shots on goal, collects the ball, and clears it upfield.
    """
    def __init__(self, robot_id: int):
        self.robot_id = robot_id

        # pass state
        self.receive_pass = False
        self.has_passed = False

        # init skills
        self.move = move.Move(robot_id)
        self.kick = kick.Kick(robot_id)
        self.receive = receive.Receive(robot_id)

    def find_wall_rad(world_state) -> float:
        # TODO: this calculation is copy-pasted from wall_tactic
        box_w = world_state.field.penalty_long_dist_m
        box_h = world_state.field.penalty_short_dist_m
        line_w = world_state.field.line_width_m
        return RobotConstants.RADIUS + line_w + np.hypot(
            box_w / 2, box_h)

    def tick(
        self,
        world_state: rc.WorldState,
        situation: ISituation
        ) -> msg.RobotIntent:

        if self.receive_pass:
            self.receive_pass = self.receive.is_done()
            return self.receive.tick(world_state)

        MIN_WALL_RAD = find_wall_rad(world_state)

        if world_state and world_state.ball.visible:
            ball_speed = np.linalg.norm(world_state.ball.vel)
            ball_dist = np.linalg.norm(world_state.field.our_goal_loc - world_state.ball.pos)

            if ball_speed < 1.0 and ball_dist < MIN_WALL_RAD - RobotConstants.RADIUS * 2.1:
                self.move_se = tactic.SkillEntry(move.Move())
                if not self.receive_se.skill.is_done(world_state): 
                    # if ball is slow and inside goalie box, collect it
                    return self.receive.tick(world_state)
                else:
                    # if ball has been stopped already, chip toward center field
                    self.kick.skill.target_point = np.array([0.0, 6.0])
                    return self.kick.tick(world_state, target_point, None)
            else:
                if ball_speed == 0:
                    ball_to_goal_time = 100
                else:
                    ball_to_goal_time = ball_dist / ball_speed

                if ball_speed > 0 and ball_to_goal_time < 2:
                    # if ball is moving and coming at goal, move laterally to block ball
                    # TODO (#1676): replace this logic with a real intercept planner
                    target_point = get_block_pt(world_state, get_goalie_pt(world_state))
                    face_point = world_state.ball.pos
                    return self.move(world_state, target_point, face_point)
                else:
                    # else, track ball normally
                    self.move_se.skill.target_point = get_goalie_pt(
                        world_state)
                    self.move_se.skill.face_point = world_state.ball.pos

        return self.receive.tick(world_state)

    def has_passed(self):
        return self.has_passed

    def receive_pass(self):
        self.receive_pass = True

    def is_done(self, world_state):
        """
        :return boolean indicating if tactic is done
        """
        # goalie tactic always active
        return False
