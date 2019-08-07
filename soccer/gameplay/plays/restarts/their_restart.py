import standard_play
import behavior
import tactics.positions.defender
import skills.mark
import skills.move
import main
import robocup
import constants

class TheirRestart(standard_play.StandardPlay):
    def __init__(self):
        super().__init__(continuous=True)

        self.debug = False

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            'immediately')

        self.marks = []

        if (main.game_state().is_their_direct()):
            self.intercept = (constants.Field.OurGoalSegment.center() -
                              main.ball().pos) * 0.25 + main.ball().pos
            self.add_subbehavior(
                skills.move.Move(self.intercept),
                'intercept direct',
                required=False,
                priority=5)

        for i in range(3):
            mark_i = skills.mark.Mark()
            mark_i.ratio = 0.7
            self.add_subbehavior(mark_i,
                                 'mark' + str(i),
                                 required=False,
                                 priority=3 - i)
            self.marks.append(mark_i)

        self.kick_eval = robocup.KickEvaluator(main.system_state())
        self.kick_eval.debug = True
        for i, robot in enumerate(main.our_robots()):
            self.kick_eval.add_excluded_robot(robot)

        for i, robot in enumerate(main.their_robots()):
            self.kick_eval.add_excluded_robot(robot)

        their_kicker = min(main.their_robots(),
                           key=lambda opp: opp.pos.dist_to(main.ball().pos))
        self.kick_eval.add_excluded_robot(their_kicker)

    @classmethod
    def score(cls):
        gs = main.game_state()
        return 0 if gs.is_ready_state() and (
            gs.is_their_free_kick() or gs.is_their_indirect() or
            gs.is_their_direct()) else float("inf")

    @classmethod
    def is_restart(cls):
        return True

    def calculate_shot_chance(self, robot):
        shot_position, success_chance = self.kick_eval.eval_pt_to_our_goal(
            robot.pos)
        if self.debug is True:
            shot_line = robocup.Segment(robot.pos, shot_position)
            main.debug_drawer().draw_line(shot_line, (0, 255, 0),
                                          "Target Position")
            main.debug_drawer().draw_text(
                "Shot Chance: " + str(success_chance),
                shot_line.center(), constants.Colors.White, "Defense")
        return success_chance

    def execute_running(self):
        super().execute_running()
        # abort if we can't see the ball
        if not main.ball().valid:
            return

        ball_pos = main.ball().pos

        self.intercept = (constants.Field.OurGoalSegment.center() -
                          main.ball().pos) * 0.25 + main.ball().pos


        # the closest of their bots to the ball is their kicker
        their_kicker = min(main.their_robots(),
                           key=lambda opp: opp.pos.dist_to(ball_pos))

        # we build an array of OpponentRobots sorted rudimentarily by their threat
        # Right now, this is (inverse of) distance to the ball * 2 + y position.
        # Needs tuning/improvement. Right now this is excessively defensive
        sorted_opponents = sorted(
            filter(lambda robot: robot != their_kicker, main.their_robots()),
            key=lambda robot: self.calculate_shot_chance(robot),
            reverse=True)

        # Decide what each marking robot should do
        # @sorted_opponents contains the robots we want to mark by priority
        # any robot that isn't assigned a mark_robot will move towards the ball
        for i, mark_i in enumerate(self.marks):
            if i < len(sorted_opponents):
                mark_i.mark_robot = sorted_opponents[i]

        self.marks[2].mark_robot = their_kicker
        self.marks[2].mark_ratio = 0.5
