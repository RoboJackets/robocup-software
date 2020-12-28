#include "window_evaluator.hpp"

#include <algorithm>
#include <array>

#include <rj_geometry/util.hpp>
#include <rj_constants/constants.hpp>

#include "debug_drawer.hpp"
#include "kick_evaluator.hpp"

REGISTER_CONFIGURABLE(WindowEvaluator)

using namespace rj_geometry;

//currently unused variable
DEFINE_NS_FLOAT64(kWindowConfigParamModule, optimization, angle_score_coefficient, 0.7,
                  "Angle score coefficient.");
DEFINE_NS_FLOAT64(kWindowConfigParamModule, optimization, distance_score_coefficient, 0.3,
                  "Distance score coefficient.");

Window::Window() : t0(0), t1(0), a0(0), a1(0), shot_success(0) {}

Window::Window(double t0, double t1) : t0(t0), t1(t1), a0(0), a1(0), shot_success(0) {}

void WindowEvaluator::create_configuration(Configuration* cfg) {
	
}

WindowEvaluator::WindowEvaluator(Context* context) : context_(context) {}

WindowingResult WindowEvaluator::eval_pt_to_pt(Point origin, Point target, float target_width) {
    auto dir_vec = (target - origin).perp_ccw().normalized();
    auto segment =
        Segment{target + dir_vec * (target_width / 2), target - dir_vec * (target_width / 2)};

    return eval_pt_to_seg(origin, segment);
}

WindowingResult WindowEvaluator::eval_pt_to_robot(Point origin, Point target) {
    return eval_pt_to_pt(origin, target, 2 * kRobotRadius);
}

WindowingResult WindowEvaluator::eval_pt_to_opp_goal(Point origin) {
    Segment their_goal{Point{-FieldDimensions::current_dimensions.goal_width() / 2,
                             FieldDimensions::current_dimensions.length()},
                       Point{FieldDimensions::current_dimensions.goal_width() / 2,
                             FieldDimensions::current_dimensions.length()}};

    return eval_pt_to_seg(origin, their_goal);
}

WindowingResult WindowEvaluator::eval_pt_to_our_goal(Point origin) {
    Segment our_goal{Point{-FieldDimensions::current_dimensions.goal_width() / 2, 0},
                     Point{FieldDimensions::current_dimensions.goal_width() / 2, 0}};

    return eval_pt_to_seg(origin, our_goal);
}

void WindowEvaluator::obstacle_range(std::vector<Window>& windows, double& t0, double& t1) {
    // Ignore degenerate obstacles
    if (t0 == t1) {
        return;
    }

    if (t0 > t1) {
        std::swap(t0, t1);
    }

    auto iter = windows.begin();
    while (iter != windows.end()) {
        auto& w = *iter;
        if (t0 <= w.t0 && t1 >= w.t1) {
            // this window is fully covered by the obstacle, so remove it
            iter = windows.erase(iter);
        } else if (t0 > w.t0 && t1 < w.t1) {
            // the window fully contains the obstacle, so we split the window
            Window w2{t1, w.t1};
            w.t1 = t0;
            iter = windows.insert(iter + 1, w2);
            iter++;
        } else if (t0 > w.t0 && t0 < w.t1) {
            // the obstacle covers the end of the window
            w.t1 = t0;
            iter++;
        } else if (t1 > w.t0 && t1 < w.t1) {
            // the obstacle covers the beginning of the window
            w.t0 = t1;
            iter++;
        } else {
            iter++;
        }
    }
}

void WindowEvaluator::obstacle_robot(std::vector<Window>& windows, Point origin, Segment target,
                                     Point bot_pos) {
    auto n = (bot_pos - origin).normalized();
    auto t = n.perp_ccw();
    auto r = kRobotRadius + kBallRadius;

    Segment seg{bot_pos - n * kRobotRadius + t * r, bot_pos - n * kRobotRadius - t * r};

    if (debug) {
        context_->debug_drawer.draw_line(seg, QColor{"Red"}, "Debug");
    }

    auto end = target.delta().magsq();

    std::array<double, 2> extent = {0, end};

    for (int i = 0; i < 2; i++) {
        Line edge{origin, seg.pt[i]};
        auto d = edge.delta().magsq();

        Point intersect;
        if (edge.intersects(Line(target), &intersect) &&
            (intersect - origin).dot(edge.delta()) > d) {
            auto f = (intersect - target.pt[0]).dot(target.delta());
            if (f < 0) {
                extent[i] = 0;
            } else if (f > end) {
                extent[i] = end;
            } else {
                extent[i] = f;
            }
        } else {
            return;
        }
    }
    obstacle_range(windows, extent[0], extent[1]);
}

WindowingResult WindowEvaluator::eval_pt_to_seg(Point origin, Segment target) {
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
}

// CDF of a normal distribution
double phi(double x) { return 0.5 * std::erf(-x / std::sqrt(2)); }

void WindowEvaluator::fill_shot_success(Window& window, Point origin) {
    auto shot_vector = window.segment.center() - origin;
    auto shot_distance = shot_vector.mag();

    // get the angle between the shot vector and the target segment, then
    // normalize and positivize it
    auto angle_between_shot_and_window = abs(shot_vector.angle() - window.segment.delta().angle());
    while (abs(angle_between_shot_and_window) > M_PI) {
        angle_between_shot_and_window -= M_PI;
    }
    angle_between_shot_and_window = abs(angle_between_shot_and_window);

    // we don't care about the segment length, we care about the width of the
    // corresponding segment perpendicular to the shot line
    auto perp_seg_length = abs(sin(angle_between_shot_and_window)) * window.segment.length();

    // the 'width' of the shot in radians
    auto angle = abs(atan2(perp_seg_length, shot_distance));

    // the wider available angle the shot has, the more likely it will make it
    // the farther the shot has to travel, the more likely that defenders can
    // block it in time
    auto shot_angle_baseline = (M_PI / 20.0);
    auto angle_score = std::min(angle / shot_angle_baseline, 1.0);

    float longest_possible_shot =
        std::sqrt(pow(FieldDimensions::current_dimensions.length(), 2.0f) +
                  pow(FieldDimensions::current_dimensions.width(), 2.0f));
    const auto& std = *KickEvaluator::kick_std_dev;
    auto angle_prob =
        phi(angle_between_shot_and_window / (std)) - phi(-angle_between_shot_and_window / (std));

    auto distance_score = 1.0 - (shot_distance / longest_possible_shot);

    window.shot_success =
        angle_prob + optimization::PARAM_distance_score_coefficient * distance_score;
}
