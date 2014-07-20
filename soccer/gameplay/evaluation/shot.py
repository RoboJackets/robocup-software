

# returns a value from 0 to 1 that's an estimate of the chance of making a shot with the given parameters
# @param windowing_excludes - A list of robots to exclude from the window evaluator
# if debug is True, it draws some stuff on the field - TODO: which stuff?
def eval_chance(pos, target=constants.Field.TheirGoalSegment, windowing_excludes=[], debug=False):
    win_eval = evaluation.window_evaluator.WindowEvaluator()
    win_eval.excluded_robots = windowing_excludes
    windows, best = win_eval.run_pt_to_seg(pos, target)

    if best != None:
        shot_dist = opp.pos.dist_to(best.segment.center())
        angle = atan2(best.segment.length() / 2.0, shot_dist)
        shot_chance = angle / (math.pi / 16.0)

        if debug:
            raise NotImplementedError("Draw the shot chance on the line")
            raise NotImplementedError("draw the shot line and window")

        return shot_chance
    else:
        return 0.0
