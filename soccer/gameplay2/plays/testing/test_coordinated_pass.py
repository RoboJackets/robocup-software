import play
import tactics.coordinated_pass
import tactics.behavior_sequence
import robocup
import constants


class TestCoordinatedPass(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        # build a list of receive points
        rcv_x = constants.Field.Width / 2.0
        field_half_len = constants.Field.Length / 2.0
        rcv_pts = [
            robocup.Point(rcv_x, field_half_len * 1.0/3.0),
            robocup.Point(-rcv_x, field_half_len * 1.0/3.0),
            robocup.Point(rcv_x, field_half_len * 2.0/3.0),
            robocup.Point(-rcv_x, field_half_len * 2.0/3.0)
        ]

        passes = [tactics.coordinated_pass.CoordinatedPass(receive_point=pt) for pt in rcv_pts]
        sequence = tactics.behavior_sequence.BehaviorSequence(passes)
        self.add_subbehavior(sequence, 'passes')
