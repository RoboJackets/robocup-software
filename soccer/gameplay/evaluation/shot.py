import constants
import evaluation.window_evaluator
import robocup


# returns a tuple (chance of shot success, best window)
# the chance of shot success is a value from 0 to 1
# @param windowing_excludes - A list of robots to exclude from the window evaluator
# if debug is True, it draws some stuff on the field - TODO: which stuff?
def eval_shot(pos, target=constants.Field.TheirGoalSegment, windowing_excludes=[], debug=False):
    win_eval = evaluation.window_evaluator.WindowEvaluator()
    win_eval.excluded_robots = windowing_excludes
    windows, best = win_eval.eval_pt_to_seg(pos, target)

    if best != None:
        shot_dist = opp.pos.dist_to(best.segment.center())
        angle = atan2(best.segment.length() / 2.0, shot_dist)

        # the wider available angle the shot has, the more likely it will make it
        # the farther the shot has to travel, the more likely that defenders can block it
        ShotAngleBaseline = (math.pi / 32.0)    # note: this angle choice is fairly arbitrary - feel free to tune it
        shot_chance = 0.6*(angle / ShotAngleBaseline) + 0.4*(constants.Field.Length / shot_dist)

        if debug:
            # raise NotImplementedError("Draw the shot chance on the line")
            # raise NotImplementedError("draw the shot line and window")
            pass

        return shot_chance, best
    else:
        return 0.0, None
