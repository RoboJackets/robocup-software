import numpy as np
import sys
sys.path.insert(1, "../../stp")
import rc
import math

class Pass: 
    ## Find the chance of a pass succeeding by looking at pass distance and what robots are in the way
    # @param from_point The Point the pass is coming from
    # @param to_point The Point the pass is being received at
    # @param excluded_robots A list of robots that shouldn't be counted as obstacles to this shot
    # @return a value from zero to one that estimates the probability of the pass succeeding
    def eval_pass(from_point,
                  to_point,
                  excluded_robots):
        if excluded_robots is None:
            excluded_robots = []
        # we make a pass triangle with the far corner at the ball and the opposing side touching the receiver's mouth
        # the side along the receiver's mouth is the 'receive_seg'
        # we then use the window evaluator on this scenario to see if the pass is open
        pass_angle = math.pi / 32.0
        #pass_dist = to_point.dist_to(from_point)
        pass_dist = sqrt(abs(from_point[0] - to_point[0])**2 + abs(from_point[1] - to_point[1])**2)
        pass_dir = to_point - from_point
        pass_perp = [-pass_dir[1], pass_dir[0]]
        receive_seg_half_len = math.tan(pass_angle) * pass_dist
        [(to_point + pass_perp * receive_seg_half_len, to_point + pass_perp * -receive_seg_half_len)]
        
        '''

        win_eval = robocup.WindowEvaluator(main.context())
        for r in excluded_robots:
            win_eval.add_excluded_robot(r)
        windows, best = win_eval.eval_pt_to_seg(from_point, receive_seg)
        
        auto end = target.delta().magsq();

    // if target is a zero-length segment, there are no windows
    if (end == 0) {
        return make_pair(std::vector<Window>{}, std::nullopt);
    }

    if (debug) {
        context_->debug_drawer.draw_line(target, QColor{"Blue"}, "Debug");
    }

    std::vector<Window> windows = {Window{0, end}};

    // apply the obstacles

    std::vector<Robot*> bots(context_->state.self.size() + context_->state.opp.size());

    auto filter_predicate = [&](const Robot* bot) -> bool {
        return bot != nullptr && bot->visible() &&
               find(excluded_robots.begin(), excluded_robots.end(), bot) == excluded_robots.end();
    };

    auto end_it = copy_if(context_->state.self.begin(), context_->state.self.end(), bots.begin(),
                          filter_predicate);

    end_it =
        copy_if(context_->state.opp.begin(), context_->state.opp.end(), end_it, filter_predicate);

    bots.resize(distance(bots.begin(), end_it));

    std::vector<Point> bot_locations;
    for_each(bots.begin(), bots.end(),
             [&bot_locations](Robot* bot) { bot_locations.push_back(bot->pos()); });

    bot_locations.insert(bot_locations.end(), hypothetical_robot_locations.begin(),
                         hypothetical_robot_locations.end());

    for (auto& pos : bot_locations) {
        auto d = (pos - origin).mag();
        // whether or not we can ship over this bot
        auto chip_overable = chip_enabled && (d < max_chip_range - kRobotRadius) &&
                             (d > min_chip_range + kRobotRadius);
        if (!chip_overable) {
            obstacle_robot(windows, origin, target, pos);
        }
    }

    auto p0 = target.pt[0];
    auto delta = target.delta() / end;

    for (auto& w : windows) {
        w.segment = Segment{p0 + delta * w.t0, p0 + delta * w.t1};
        w.a0 = radians_to_degrees(static_cast<float>((w.segment.pt[0] - origin).angle()));
        w.a1 = radians_to_degrees(static_cast<float>((w.segment.pt[1] - origin).angle()));
        fill_shot_success(w, origin);
    }

    std::optional<Window> best;
    if (!windows.empty()) {
        best = *max_element(windows.begin(), windows.end(), [](Window& a, Window& b) -> bool {
            return a.segment.delta().magsq() < b.segment.delta().magsq();
        });
    }
    if (debug) {
        if (best) {
            context_->debug_drawer.draw_line(Segment{origin, best->segment.center()},
                                             QColor{"Green"}, "Debug");
        }
        for (Window& window : windows) {
            context_->debug_drawer.draw_line(window.segment, QColor{"Green"}, "Debug");
            context_->debug_drawer.draw_text(QString::number(window.shot_success),
                                             window.segment.center() + Point(0, 0.1),
                                             QColor{"Green"}, "Debug");
        }
    }

    return make_pair(windows, best);

        # this is our estimate of the likelihood of the pass succeeding
        # value can range from zero to one
        # we square the ratio of best to total to make it weigh more - we could raise it to higher power if we wanted
        if best != None:
            return 0.8 * (best.segment.length() / receive_seg.length())**2
        else:
            # the pass is completely blocked
            return 0
        '''
        return 0
