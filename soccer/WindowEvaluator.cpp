#include "WindowEvaluator.hpp"
#include "Constants.hpp"
#include <Geometry2d/Util.hpp>

#include <algorithm>
#include <array>
#include <iostream>

REGISTER_CONFIGURABLE(WindowEvaluator)

using namespace std;
using namespace Geometry2d;

ConfigDouble* WindowEvaluator::angle_score_coefficient;
ConfigDouble* WindowEvaluator::distance_score_coefficient;

Window::Window() : t0(0), t1(0), a0(0), a1(0), shot_success(0) {}

Window::Window(double t0, double t1)
    : t0(t0), t1(t1), a0(0), a1(0), shot_success(0) {}

void WindowEvaluator::createConfiguration(Configuration* cfg) {
    angle_score_coefficient =
        new ConfigDouble(cfg, "WindowEvaluator/angleScoreCoeff", 0.7);
    distance_score_coefficient =
        new ConfigDouble(cfg, "WindowEvaluator/distScoreCoeff", 0.3);
}

WindowEvaluator::WindowEvaluator(SystemState* systemState)
    : system(systemState) {}

WindowingResult WindowEvaluator::eval_pt_to_pt(Point origin, Point target,
                                               float targetWidth) {
    auto dir_vec = (target - origin).perpCCW().normalized();
    auto segment = Segment{target + dir_vec * (targetWidth / 2),
                           target - dir_vec * (targetWidth / 2)};

    return eval_pt_to_seg(origin, segment);
}

WindowingResult WindowEvaluator::eval_pt_to_robot(Point origin, Point target) {
    return eval_pt_to_pt(origin, target, 2 * Robot_Radius);
}

WindowingResult WindowEvaluator::eval_pt_to_opp_goal(Point origin) {
    Segment their_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()}};

    return eval_pt_to_seg(origin, their_goal);
}

WindowingResult WindowEvaluator::eval_pt_to_our_goal(Point origin) {
    Segment our_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0}};

    return eval_pt_to_seg(origin, our_goal);
}

void WindowEvaluator::obstacle_range(vector<Window>& windows, double& t0,
                                     double& t1) {
    // Ignore degenerate obstacles
    if (t0 == t1) return;

    if (t0 > t1) swap(t0, t1);

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

void WindowEvaluator::obstacle_robot(vector<Window>& windows, Point origin,
                                     Segment target, Point bot_pos) {
    auto n = (bot_pos - origin).normalized();
    auto t = n.perpCCW();
    auto r = Robot_Radius + Ball_Radius;

    Segment seg{bot_pos - n * Robot_Radius + t * r,
                bot_pos - n * Robot_Radius - t * r};

    if (debug) {
        system->drawLine(seg, QColor{"Red"}, "Debug");
    }

    auto end = target.delta().magsq();

    array<double, 2> extent = {0, end};

    for (int i = 0; i < 2; i++) {
        Line edge{origin, seg.pt[i]};
        auto d = edge.delta().magsq();

        Point intersect;
        if (edge.intersects(Line(target), &intersect) &&
            (intersect - origin).dot(edge.delta()) > d) {
            auto f = (intersect - target.pt[0]).dot(target.delta());
            if (f < 0)
                extent[i] = 0;
            else if (f > end)
                extent[i] = end;
            else
                extent[i] = f;
        } else {
            return;
        }
    }
    obstacle_range(windows, extent[0], extent[1]);
}

WindowingResult WindowEvaluator::eval_pt_to_seg(Point origin, Segment target) {
    auto end = target.delta().magsq();

    // if target is a zero-length segment, there are no windows
    if (end == 0) return make_pair(vector<Window>{}, boost::none);

    if (debug) {
        system->drawLine(target, QColor{"Blue"}, "Debug");
    }

    vector<Window> windows = {Window{0, end}};

    // apply the obstacles

    vector<Robot*> bots(system->self.size() + system->opp.size());

    auto filter_predicate = [&](const Robot* bot) -> bool {
        return bot != nullptr && bot->visible &&
               find(excluded_robots.begin(), excluded_robots.end(), bot) ==
                   excluded_robots.end();
    };

    auto end_it = copy_if(system->self.begin(), system->self.end(),
                          bots.begin(), filter_predicate);

    end_it = copy_if(system->opp.begin(), system->opp.end(), end_it,
                     filter_predicate);

    bots.resize(distance(bots.begin(), end_it));

    vector<Point> bot_locations;
    for_each(bots.begin(), bots.end(), [&bot_locations](Robot* bot) {
        bot_locations.push_back(bot->pos);
    });

    bot_locations.insert(bot_locations.end(),
                         hypothetical_robot_locations.begin(),
                         hypothetical_robot_locations.end());

    for (auto& pos : bot_locations) {
        auto d = (pos - origin).mag();
        // whether or not we can ship over this bot
        auto chip_overable = chip_enabled &&
                             (d < max_chip_range - Robot_Radius) &&
                             (d > min_chip_range + Robot_Radius);
        if (!chip_overable) {
            obstacle_robot(windows, origin, target, pos);
        }
    }

    auto p0 = target.pt[0];
    auto delta = target.delta() / end;

    for (auto& w : windows) {
        w.segment = Segment{p0 + delta * w.t0, p0 + delta * w.t1};
        w.a0 = RadiansToDegrees((w.segment.pt[0] - origin).angle());
        w.a1 = RadiansToDegrees((w.segment.pt[1] - origin).angle());
        fill_shot_success(w, origin);
    }

    boost::optional<Window> best{
        !windows.empty(), *max_element(windows.begin(), windows.end(),
                                       [](Window& a, Window& b) -> bool {
                                           return a.segment.delta().magsq() <
                                                  b.segment.delta().magsq();
                                       })};
    if (debug) {
        if (best) {
            system->drawLine(Segment{origin, best->segment.center()},
                             QColor{"Green"}, "Debug");
        }
        for (Window& window : windows) {
            system->drawLine(window.segment, QColor{"Green"}, "Debug");
            system->drawText(QString::number(window.shot_success),
                             window.segment.center() + Point(0, 0.1),
                             QColor{"Green"}, "Debug");
        }
    }

    return make_pair(windows, best);
}

void WindowEvaluator::fill_shot_success(Window& window, Point origin) {
    auto shot_vector = window.segment.center() - origin;
    auto shot_distance = shot_vector.mag();

    // get the angle between the shot vector and the target segment, then
    // normalize and positivize it
    auto angle_between_shot_and_window =
        abs(shot_vector.angle() - window.segment.delta().angle());
    while (abs(angle_between_shot_and_window) > M_PI) {
        angle_between_shot_and_window -= M_PI;
    }
    angle_between_shot_and_window = abs(angle_between_shot_and_window);

    // we don't care about the segment length, we care about the width of the
    // corresponding segment perpendicular to the shot line
    auto perp_seg_length =
        abs(sin(angle_between_shot_and_window)) * window.segment.length();

    // the 'width' of the shot in radians
    auto angle = abs(atan2(perp_seg_length, shot_distance));

    // the wider available angle the shot has, the more likely it will make it
    // the farther the shot has to travel, the more likely that defenders can
    // block it in time
    auto shot_angle_baseline = (M_PI / 20.0);
    auto angle_score = min(angle / shot_angle_baseline, 1.0);

    float longest_possible_shot =
        std::sqrt(pow(Field_Dimensions::Current_Dimensions.Length(), 2.0f) +
                  pow(Field_Dimensions::Current_Dimensions.Width(), 2.0f));

    auto distance_score = 1.0 - (shot_distance / longest_possible_shot);

    window.shot_success = *angle_score_coefficient * angle_score +
                          *distance_score_coefficient * distance_score;
}
