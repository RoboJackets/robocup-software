import play
import behavior
import tactics.coordinated_pass
import tactics.forward_pass
import robocup
import constants
import main


## Continually runs a coordinated pass to opposite sides of the field
class TestForwardPass(play.Play):

    ReceiveXCoord = -constants.Field.Width / 4
    ReceiveYCoord = constants.Field.Length * 3 / 4.0

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        pass_bhvr = tactics.forward_pass.ForwardPass()
        self.add_subbehavior(pass_bhvr, 'pass')

    def reset_receive_point(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        #x = TestForwardPass.ReceiveXCoord if main.ball(
        #).pos.x < 0 else -TestForwardPass.ReceiveXCoord
        pass_bhvr.receive_point = robocup.Point(TestForwardPass.ReceiveXCoord,
                                                TestForwardPass.ReceiveYCoord)

    def execute_running(self):
        pass_bhvr = self.subbehavior_with_name('pass')

        if pass_bhvr.is_done_running():
            pass_bhvr.restart()
            self.reset_receive_point()

        if pass_bhvr.receive_point == None:
            self.reset_receive_point()
