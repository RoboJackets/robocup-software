import play
import behavior
import tactics.coordinated_pass
import tactics.behavior_sequence
import robocup
import constants
import main


class TestCoordinatedPass(play.Play):

    ReceiveXCoord = constants.Field.Width * 1.0/3.0
    ReceiveYCoord = constants.Field.Length / 2.0 * 1.0/3.0

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')

        pass_bhvr = tactics.coordinated_pass.CoordinatedPass()
        self.add_subbehavior(pass_bhvr, 'pass')


    def execute_running(self):
        pass_bhvr = self.subbehavior_with_name('pass')

        if pass_bhvr.is_done_running():
            pass_bhvr.restart()

        if pass_bhvr.receive_point == None:
            x = TestCoordinatedPass.ReceiveXCoord if main.ball().pos.x < 0 else -TestCoordinatedPass.ReceiveXCoord
            pass_bhvr.receive_point = robocup.Point(x, TestCoordinatedPass.ReceiveYCoord)
            