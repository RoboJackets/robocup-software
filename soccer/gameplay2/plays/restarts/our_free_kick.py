import play
import behavior
import skills.move
import skills.pivot_kick
import tactics.positions.fullback
import constants
import robocup


class OurFreeKick(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        # FIXME: this could also be a PivotKick
        kicker = skills.line_kick.LineKick()
        kicker.use_chipper = True
        self.add_subbehavior(kicker, 'kicker', required=False, priority=5)


        # add two 'centers' that just move to fixed points
        center1 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center1, 'center1', required=False, priority=4)
        center2 = skills.move.Move(robocup.Point(0, 1.5))
        self.add_subbehavior(center1, 'center2', required=False, priority=3)

        fullback1 = tactics.positions.fullback.Fullback(side=tactics.positions.fullback.Fullback.Side.left)
        self.add_subbehavior(fullback1, 'fullback1', required=False, priority=2)

        fullback2 = tactics.positions.fullback.Fullback(side=tactics.positions.fullback.Fullback.Side.left)
        self.add_subbehavior(fullback2, 'fullback2', required=False, priority=1)



    @classmethod
    def score(cls):
        gs = main.game_state()
        return 10 if gs.is_setup_state() and gs.is_our_freekick() else float("inf")



    def run(self):
        # TODO: set kicker's minchiprange(0.3) and maxchiprange(2)
        # TODO: set kicker's goal
        #        prev implementation set it to goal line, right downfield, or left downfield

        kicker = self.subbehavior_with_name('kicker')
        if kicker.robot != None and kicker.robot.pos.near_point(main.ball().pos, 0.3):
            pass
            # FIXME: add shot channel obstacle for the other robots
