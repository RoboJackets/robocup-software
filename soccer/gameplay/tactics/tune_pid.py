import single_robot_composite_behavior
import behavior
import main
import constants
import role_assignment
import enum
import robocup
import skills
import skills.move
import time


class Tune_pid(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        tune = 1
        process=2

    def __init__(self):
        super().__init__(continuous=False)

        self.pause_time = 0

        for substate in Tune_pid.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, Tune_pid.State.tune, lambda: True,'immediately')


        self.add_transition(Tune_pid.State.tune, Tune_pid.State.process,lambda: self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,'finished moving')


        self.add_transition(Tune_pid.State.process, Tune_pid.State.tune,lambda: not self.check_complete(),'continue tuning')

        self.add_transition(Tune_pid.State.process, behavior.Behavior.State.completed, lambda: self.check_complete(), 'done tuning')

        self.positions=[]

        xsize = constants.Field.Width/2

        self.left_point=robocup.Point(-xsize,2)
        self.right_point=robocup.Point(xsize,2)
        self.line = robocup.Segment(self.left_point,self.right_point)

    def check_complete(self):
        return False;

    def on_enter_tune(self):
        if((self.robot.pos-self.left_point).mag() < .1):
            move = skills.move.Move(self.right_point)
            print("go right")
        else:
            move= skills.move.Move(self.left_point)
            print("go left")

        self.robot.set_pid(100,100,100)

        self.add_subbehavior(move, 'move', required=True, priority=100)

    def execute_tune(self):
        pos=self.robot.pos
        self.positions.append(pos)

    def on_exit_tune(self):
        print("EXIT")
        print("LENGTH: ",len(self.positions))
        #print(self.positions)
        self.remove_subbehavior('move')

    def on_enter_process(self):
        print("process stuff")
        self.positions=[]

    def role_requirements(self):
        reqs = super().role_requirements()
        if isinstance(reqs, role_assignment.RoleRequirements):
            reqs.required_shell_id = 0

        return reqs
