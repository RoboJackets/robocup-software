import play
import behavior
import skills.move
import tactics.coordinated_pass
import tactics.behavior_sequence
import robocup
import constants
import main
import skills.angle_receive
from enum import Enum

# A play to test onetouchpass, which causes a robot to pass to another one,
# who scores on the goal as fast as possible.
#
# The real work is in the class that receives from coordinated_pass, angle_receive
class OneTouchPass(play.Play):

    ReceiveXCoord = 1
    ReceiveYCoord = constants.Field.Length * 5.0 / 6.0

    class State(Enum):
        passing = 1

    def __init__(self):
        super().__init__(continuous=False)

        for state in OneTouchPass.State:
            self.add_state(state, behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
                OneTouchPass.State.passing,
                lambda: True,
                'immediately')

        self.add_transition(OneTouchPass.State.passing,
                behavior.Behavior.State.completed,
                lambda: self.subbehavior_with_name('pass').state == behavior.Behavior.State.completed,
                'preparing to shoot')

    def reset_receive_point(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        pass_bhvr.receive_point = robocup.Point(OneTouchPass.ReceiveXCoord, OneTouchPass.ReceiveYCoord)
        pass_bhvr.skillreceiver = skills.angle_receive.AngleReceive()

    def on_enter_passing(self):
        pass_bhvr = tactics.coordinated_pass.CoordinatedPass()
        self.add_subbehavior(pass_bhvr, 'pass')
        if pass_bhvr.receive_point == None:
            self.reset_receive_point()

    def on_exit_passing(self):
        self.remove_subbehavior('pass')

