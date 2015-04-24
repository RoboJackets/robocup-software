import play
import behavior
import tactics.coordinated_pass
import tactics.behavior_sequence
import robocup
import constants
import main


## Continually runs a coordinated pass to opposite sides of the field
class TestCoordinatedPass(play.Play):

    ReceiveXCoord = 1
    ReceiveYCoord = constants.Field.Length / 2.0 * 1.0/3.0

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        pass_bhvr = tactics.coordinated_pass.CoordinatedPass()
        self.add_subbehavior(pass_bhvr, 'pass')



    def reset_receive_point(self):
        pass_bhvr = self.subbehavior_with_name('pass')
        x = TestCoordinatedPass.ReceiveXCoord if main.ball().pos.x < 0 else -TestCoordinatedPass.ReceiveXCoord
        pass_bhvr.receive_point = robocup.Point(x, TestCoordinatedPass.ReceiveYCoord)


    def execute_running(self):
        pass_bhvr = self.subbehavior_with_name('pass')

        if pass_bhvr.is_done_running():
            self.remove_subbehavior('pass')
            pass_bhvr = tactics.coordinated_pass.CoordinatedPass()
            self.add_subbehavior(pass_bhvr, 'pass')
            self.reset_receive_point()

        if pass_bhvr.receive_point == None:
            self.reset_receive_point()

