import play
import behavior
import tactics.coordinated_pass
import tactics.behavior_sequence
import skills.moving_pass_receive
import robocup
import constants
import main


## Continually runs a coordinated pass to opposite sides of the field
class TestThroughPass(play.Play):

    ReceiveXCoord = 1
    ReceiveYCoord = constants.Field.Length / 2.0 * 1.0 / 3.0

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        pass_bhvr = tactics.coordinated_pass.CoordinatedPass(robocup.Point(TestThroughPass.ReceiveXCoord, TestThroughPass.ReceiveYCoord), skills.moving_pass_receive.MovingPassReceive())
        self.add_subbehavior(pass_bhvr, 'pass')

    def reset_receive_point(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        
        if (main.ball().pos.x < 0):
            x = TestThroughPass.ReceiveXCoord 
        else:
            x = -TestThroughPass.ReceiveXCoord

        pass_bhvr.receive_point = robocup.Point(x, TestThroughPass.ReceiveYCoord)

    def execute_running(self):
        pass_bhvr = self.subbehavior_with_name('pass')

        if pass_bhvr.is_done_running():
            pass_bhvr.restart()
            self.reset_receive_point()

        if pass_bhvr.receive_point == None:
            self.reset_receive_point()
