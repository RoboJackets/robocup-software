import play
import behavior
import robocup
import skills
import main
import constants


class OurKickoff(play.Play):

    KickPower = 127
    ChipPower = 100


    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        # FIXME: reconfigure this once Kickoff behavior is written
        # kicker = skills.kickoff.Kickoff()
        # kicker.target = robocup.Segment(robocup.Point(-constants.Field.Width/2.0, constants.Field.Length),
        #     robocup.Point(constants.Field.Width/2.0, constants.Field.Length))
        # kicker.use_chipper = True
        # kicker.kick_power = OurKickoff.KickPower
        # kicker.chip_power = OurKickoff.ChipPower
        # self.add_subbehavior(kicker, 'kicker', required=True, priority=5)


        # TODO: verify that these values are right - I'm fuzzy on my matrix multiplication...
        idle_positions = [
            robocup.Point(0.7, constants.Field.Length / 2.0 - 0.2),
            robocup.Point(-0.7, constants.Field.Length / 2.0 - 0.2),
            robocup.Point(0.2, 1.5),
            robocup.Point(-0.2, 1.5)
        ]
        self.centers = []
        for i, pos_i in enumerate(idle_positions):
            center_i = skills.move.Move(pos_i)
            self.add_subbehavior(center_i, 'center' + str(i), required=False, priority=4-i)
            self.centers.append(center_i)



    def score(self):
        gs = main.game_state()
        return 0 if gs.is_setup_state() and gs.is_our_kickoff() else float("inf")



    def execute_running(self):
        # all centers should face the ball
        for center in self.centers:
            center.face(main.ball().pos)
