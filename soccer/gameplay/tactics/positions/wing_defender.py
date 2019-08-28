import single_robot_behavior
import behavior
import robocup
import math
import main
import skills
import enum
import constants
import role_assignment
import evaluation.defensive_positioning
from skills.move import Move

class WingDefender(single_robot_behavior.SingleRobotBehavior):

    class State(enum.Enum):
        defending = 1
        intercepting = 2

    def __init__(self,
                 goalside_ratio = .5,
                 distance = .5,
                 mark_robot = None,
                 mark_point = None):
        super().__init__(continuous=True)

        self._goalside_ratio = goalside_ratio   # Ratio of angle defending goal line versus attacker line
        self._mark_robot = mark_robot           # Robot we are defending against
        self._distance = distance if distance >= 2*constants.Robot.Radius else 2*constants.Robot.Radius   # Distance from the point we are defending against

        self._mark_pos = mark_point if mark_point != None \
            else (mark_robot.pos if self.mark_robot != None else None)

        self.kick_eval = robocup.KickEvaluator(main.system_state())

        self.add_state(WingDefender.State.defending, 
                       behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                            WingDefender.State.defending, lambda: True,
                            'immediately')
        self.mv_pt = self.calc_move_point()
        
        # self.add_transition(
        #   WingDefender.State.defending, WingDefender.State.intercepting,
        #   lambda: self.ball_kicked,
        #   'ball kicked, moving to intercept')

    def execute_defending(self):
        self._mark_pos = self._mark_robot.pos if self._mark_robot != None else self.mark_pos
        self.mv_pt = self.calc_move_point()
        self.robot.move_to(self.mv_pt)

    def calc_move_point(self):
        if self._mark_pos != None: 
            goal_line, shot_pt = evaluation.defensive_positioning.goalside_mark_segment(self._mark_pos, self.robot, ball=False, kick_eval=self.kick_eval)
            ballside_line = evaluation.defensive_positioning.ballside_mark_segment(self._mark_pos, main.ball().pos)
            total_angle = (main.ball().pos - self._mark_pos).angle_between(shot_pt - self._mark_pos)
            goal_angle = (main.ball().pos - self._mark_pos).normalized().dot((shot_pt - self._mark_pos).normalized())
            #If opposing robot is between goal and ball, either get on goalside or mark side -
            # not really a wing defender anymore- this is a hacky fix
            if goal_angle < -.9:
                #print("Striker")
                if (self._mark_pos - main.ball().pos).mag() > (constants.TheirChipping[1] + constants.Robot.Radius*2):
                    #print("Far Away")
                    self._goalside_ratio = 1
                    
                else:
                    self._goalside_ratio = 0

            angle = total_angle * self._goalside_ratio
            norm_goal_line = ((shot_pt - self._mark_pos).normalized()*self.distance + self._mark_pos)
            angle = -1 * angle if main.ball().pos.x - self._mark_pos.x < 0 else angle       
            norm_goal_line.rotate(self._mark_pos, angle)
            return norm_goal_line

    @property
    def mark_pos(self):
        return self._mark_pos
    
    @mark_pos.setter
    def mark_pos(self, point):
        self._mark_pos = point
        self._mark_robot = None

    @property
    def goalside_ratio(self):
        return self._goalside_ratio

    @goalside_ratio.setter
    def goalside_ratio(self, value):
        self._goalside_ratio = value

    @property
    def distance(self):
        return self._distance

    @distance.setter
    def distance(self, value):
        self._distance = value
    

    @property
    def mark_robot(self):
        return self._mark_robot
    
    @mark_robot.setter
    def mark_robot(self, robot):
        self._mark_robot = robot

    def role_requirements(self):
        reqs = super().role_requirements()
        reqs.destination_shape = self.mv_pt
        return reqs