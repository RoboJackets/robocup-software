import play
import behavior
import tactics.positions.fullback
import skills.mark


class TheirFreekick(play.Play):

    def __init__(self):
        super().__init__(continuous=True)

        self.add_transition(behavior.Behavior.State.start,
            behavior.Behavior.State.running,
            lambda: True,
            'immediately')


        fullback1 = tactics.positions.fullback.Fullback(tactics.positions.fullback.Fullback.Side.left)
        self.add_subbehavior(fullback1, 'fullback1', required=False, priority=5)

        fullback2 = tactics.positions.fullback.Fullback(tactics.positions.fullback.Fullback.Side.right)
        self.add_subbehavior(fullback2, 'fullback2', required=False, priority=4)

        self.marks = []
        for i in range(3):
            mark_i = skills.mark.Mark()
            mark_i.ratio = 0.7
            self.add_subbehavior(mark_i, 'mark' + str(i), required=False, priority=3 - i)
            self.marks.append(mark_i)


    def score(self):
        gs = main.game_state()
        return 0 if gs.is_setup_state() and gs.is_their_freekick() else float("inf")


    def execute_running(self):
        # abort if we can't see the ball
        if not main.ball().valid:
            return


        ball_pos = main.ball().pos


        # the closest of their bots to the ball is their kicker
        their_kicker = min(main.their_robots(), key=lambda opp: opp.pos.dist_to(ball_pos))


        # we build an array of (OpponentRobot, float distToClosestOfOurBots) tuples
        # we'll use these in a sec to assign our marking robots
        open_opps_and_dists = []
        for opp in main.their_robots():
            # ignore their kicker
            if opp == their_kicker: continue

            ball_dist = opp.pos.dist_to(ball_pos)

            # see if the opponent is close enough to the ball for us to care
            # if it is, we record the closest distance from one of our robots to it
            if ball_dist < 3.0:
                # which of our robots is closest to this opponent
                closest_self_dist = min([bot.pos.distTo(opp.pos) for bot in main.our_robots()])
                open_opps_and_dists.append( (opp, closest_self_dist) )


        # Decide what each marking robot should do
        # @open_opps contains the robots we want to mark (if any)
        # any robot that isn't assigned a mark_robot will move towards the ball
        # note: subbehaviors are executed before their superbehaviors, so the move() and face() here override the mark behavior's normal control
        for i, mark_i in enumerate(self.marks):
            if mark_i.robot != None:
                if i < len(open_opps_and_dists):
                    # mark the opponent
                    mark_i.mark_robot = open_opps_and_dists[i](0)
                else:
                    # move towards the ball and face it, but don't get within field center's radius
                    pos = mark_i.robot.pos
                    target = pos + (ball_pos - pos).normalized() * (ball_pos.dist_to(pos) - constants.Field.CenterRadius)
                    mark_i.robot.move(target)
                    mark_i.face(ball_pos)


        # tell the marking robots to avoid eachother more than normal
        for i, mark_i in enumerate(self.marks):
            for j, mark_j in enumerate(self.marks):
                if i == j: continue
                if mark_i.robot != None and mark_j.robot != None:
                    mark_i.robot.set_avoid_teammate_radius(mark_j.shell_id(), 0.5)
