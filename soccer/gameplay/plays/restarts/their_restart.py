import standard_play
import behavior
import tactics.positions.defender
import skills.mark
import main


class TheirRestart(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.marks = []
        for i in range(3):
            mark_i = skills.mark.Mark()
            mark_i.ratio = 0.7
            self.add_subbehavior(mark_i,
                                 'mark' + str(i),
                                 required=False,
                                 priority=3 - i)
            self.marks.append(mark_i)

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_ready_state() and (
            gs.is_their_free_kick() or gs.is_their_indirect() or
            gs.is_their_direct()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def execute_running(self):
        super().execute_running()
        # abort if we can't see the ball
        if not main.ball().valid:
            return

        ball_pos = main.ball().pos

        # the closest of their bots to the ball is their kicker
        their_kicker = min(main.their_robots(),
                           key=lambda opp: opp.pos.dist_to(ball_pos))

        # we build an array of OpponentRobots sorted rudimentarily by their threat
        # Right now, this is (inverse of) distance to the ball * 2 + y position.
        # Needs tuning/improvement. Right now this is excessively defensive
        sorted_opponents = sorted(
            filter(lambda robot: robot != their_kicker, main.their_robots()),
            key=lambda robot: robot.pos.dist_to(ball_pos) * 2 + robot.pos.y)

        # Decide what each marking robot should do
        # @sorted_opponents contains the robots we want to mark by priority
        # any robot that isn't assigned a mark_robot will move towards the ball
        for i, mark_i in enumerate(self.marks):
            if i < len(sorted_opponents):
                mark_i.mark_robot = sorted_opponents[i]
