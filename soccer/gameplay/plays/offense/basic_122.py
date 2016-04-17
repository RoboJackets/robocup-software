import play
import behavior
import skills
import tactics
import main
import robocup
import evaluation
import constants
import math


class Basic122(play.Play):

    # how far the 2 support robots should stay away from the striker
    SupportAvoidTeammateRadius = 0.5

    OffenseSupportRatio = 0.7
    DefenseSupportRatio = 0.9

    # if an opponent is this close to the striker, we don't instruct the support bots to mark him
    SupportBackoffThresh = 1.5

    # multiplier used to decide when it's worth it to reassign the bot a support is marking
    MarkHysteresisCoeff = 0.9

    def __init__(self):
        super().__init__(continuous=False)  # FIXME: continuous?

        striker = skills.pivot_kick.PivotKick()
        striker.aim_params['error_threshold'] = 0.15
        striker.aim_params['max_steady_ang_vel'] = 7
        striker.aim_params['min_steady_duration'] = 0.1
        striker.aim_params['desperate_timeout'] = 2.5

        self.add_transition(behavior.Behavior.State.start,
                            behavior.Behavior.State.running, lambda: True,
                            "immediately")

        striker.add_transition(behavior.Behavior.State.completed,
                               behavior.Behavior.State.start, lambda: True,
                               "immediately")
        self.add_subbehavior(striker, 'striker', required=False, priority=3)

        support1 = skills.mark.Mark()
        support1.mark_line_thresh = 1.0
        self.add_subbehavior(support1, 'support1', required=False, priority=2)

        support2 = skills.mark.Mark()
        support2.mark_line_thresh = 1.0
        self.add_subbehavior(support2, 'support2', required=False, priority=1)

        self.add_subbehavior(tactics.defense.Defense(),
                             'defense',
                             required=False)

    @classmethod
    def score(cls):
        return 10 if main.game_state().is_playing() else float("inf")

    @classmethod
    def handles_goalie(cls):
        return True

    def execute_running(self):
        striker = self.subbehavior_with_name('striker')
        support1 = self.subbehavior_with_name('support1')
        support2 = self.subbehavior_with_name('support2')
        supports = [support1, support2]

        # project ball location a bit into the future
        ball_proj = evaluation.ball.predict(main.ball().pos,
                                            main.ball().vel,
                                            t=0.75)

        # find closest opponent to striker
        closest_dist_to_striker, closest_opp_to_striker = float("inf"), None
        if striker.robot != None:
            for opp in main.their_robots():
                d = opp.pos.dist_to(striker.robot.pos)
                if d < closest_dist_to_striker:
                    closest_dist_to_striker, closest_opp_to_striker = d, opp

        striker_engaged = striker.robot != None and closest_dist_to_striker < Basic122.SupportBackoffThresh

        # pick out which opponents our defenders should 'mark'
        # TODO: explain
        nrOppClose = 0
        bestOpp1, bestOpp2 = None, None
        bestDistSq1, bestDistSq2 = float("inf"), float("inf")
        for opp in main.their_robots():
            # use dist from goal rather than just y-coord to handle corners better
            oppDistSq = opp.pos.magsq()
            if not (striker_engaged and opp == closest_opp_to_striker):
                if oppDistSq < bestDistSq1:
                    bestDistSq2, bestOpp2 = bestDistSq1, bestOpp1
                    bestDistSq1, bestOpp1 = oppDistSq, opp
                elif oppDistSq < bestDistSq2:
                    bestDistSq2, bestOpp2 = oppDistSq, opp

                if oppDistSq < constants.Field.Length**2 / 4.0:
                    nrOppClose += 1

        # handle the case of having no good robots to mark
        if bestOpp1 == None and support1.robot != None:
            support1.mark_robot = None
            support1.robot.add_text("No mark target", (255, 255, 255),
                                    "RobotText")
            if striker.robot != None:
                support1.robot.add_text("Static Support", (255, 255, 255),
                                        "RobotText")
                support_goal = striker.robot.pos
                support_goal.x *= -1.0
                if abs(support_goal.x) < 0.2:
                    support_goal.x = -1.0 if support_goal.x < 0 else 1.0

                if ball_proj.y > constants.Field.Length / 2.0 and nrOppClose > 0:
                    support_goal.y = max(support_goal.y *
                                         Basic122.OffenseSupportRatio, 0.3)
                else:
                    support_goal.y = max(support_goal.y *
                                         Basic122.DefenseSupportRatio, 0.3)

                support1.robot.move_to(support_goal)
                support1.robot.face(ball_proj)

        # sit around and do jack shit...
        # TODO: make it do something useful
        if bestOpp2 == None and support2.robot != None:
            support2.mark_robot = None
            support2.robot.add_text("No mark target", (255, 255, 255),
                                    "RobotText")
            if striker.robot != None:
                support2.robot.add_text("Static Support", (255, 255, 255),
                                        "RobotText")
                support_goal = striker.robot.pos
                support_goal.x *= -1.0
                if abs(support_goal.x) < 0.2:
                    support_goal.x = -1.0 if support_goal.x < 0 else 1.0

                if ball_proj.y > constants.Field.Length / 2.0 and nrOppClose > 0:
                    support_goal.y = max(support_goal.y *
                                         Basic122.OffenseSupportRatio, 0.3)
                else:
                    support_goal.y = max(support_goal.y *
                                         Basic122.DefenseSupportRatio, 0.3)

                support2.robot.move_to(support_goal)
                support2.robot.face(ball_proj)

        # reassign support robots' mark targets based on dist sq and hysteresis coeff
        new_dists = [bestDistSq1, bestDistSq2]
        new_bots = [bestOpp1, bestOpp2]
        for i in range(2):
            support = supports[i]
            if new_bots[i] != None:
                cur_dist_sq = (
                    support.mark_robot.pos -
                    ball_proj).magsq() if support.mark_robot else float("inf")
                if new_dists[i] < cur_dist_sq * Basic122.MarkHysteresisCoeff:
                    support.mark_robot = new_bots[i]

        # if the supports are farther from the ball, they can mark further away
        if ball_proj.y > constants.Field.Length / 2.0 and nrOppClose > 0:
            for support in supports:
                support.ratio = Basic122.OffenseSupportRatio
        else:
            for support in supports:
                support.ratio = Basic122.DefenseSupportRatio

        # keep support robots away from the striker
        if striker.robot != None:
            for supp in [support1, support2]:
                if supp.robot != None:
                    supp.robot.set_avoid_teammate_radius(
                        striker.robot.shell_id(),
                        Basic122.SupportAvoidTeammateRadius)

            # raise NotImplementedError("Make support robots avoid the shot channel")
            # FROM C++:
            # Polygon shot_obs;
            # shot_obs.vertices.push_back(Geometry2d::Point(Field_GoalWidth / 2, Field_Length));
            # shot_obs.vertices.push_back(Geometry2d::Point(-Field_GoalWidth / 2, Field_Length));
            # shot_obs.vertices.push_back(ballProj);

            # TODO: this would be a good place for a "keep-trying container behavior"
            # make the kicker try again if it already kicked
        if not striker.is_in_state(behavior.Behavior.State.running):
            striker.restart()
