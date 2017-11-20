import single_robot_composite_behavior
import behavior
import main
import constants
import role_assignment
import enum
import robocup
import skills
import skills.move
import skills.move_direct
import skills.move_tuning
import time


class Tune_pid(single_robot_composite_behavior.SingleRobotCompositeBehavior):
    class State(enum.Enum):
        tune = 1
        process = 2

    def __init__(self):
        super().__init__(continuous=False)

        #TODO: find a better way to do this so it can be a sub 1-second pause
        self.pause = 0

        for substate in Tune_pid.State:
            self.add_state(substate, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start, Tune_pid.State.tune,
                            lambda: True, 'immediately')

        self.add_transition(
            Tune_pid.State.tune, Tune_pid.State.process,
            lambda: self.subbehavior_with_name('move').state == behavior.Behavior.State.completed,
            'finished moving')

        self.add_transition(Tune_pid.State.process, Tune_pid.State.tune,
                            lambda: self.tune and time.time() - self.pause > 1,
                            'continue tuning')

        self.add_transition(Tune_pid.State.process,
                            behavior.Behavior.State.completed,
                            lambda: not self.tune, 'done tuning')

        xsize = constants.Field.Width / 10

        self.left_point = robocup.Point(-xsize, 2)
        self.right_point = robocup.Point(xsize, 2)

        self.tune = True

        self.pause = 0

    def on_enter_running(self):
        self.robot.initialize_tuner('x')

    def on_enter_tune(self):
        if (self.robot.pos.x < 0):
            move = skills.move_tuning.MoveTuning(self.right_point)
        else:
            move = skills.move_tuning.MoveTuning(self.left_point)

        move.check_velocity = True
        self.robot.start_pid_tuner('x')

        self.add_subbehavior(move, 'move', required=True, priority=100)

    def execute_tune(self):
        self.robot.run_pid_tuner('x')

    def on_exit_tune(self):
        self.remove_subbehavior('move')
        self.tune = self.robot.end_pid_tuner('x')

    def on_enter_process(self):
        self.pause = time.time()
