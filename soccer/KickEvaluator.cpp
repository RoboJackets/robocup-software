#include "KickEvaluator.hpp"
#include <Geometry2d/Util.hpp>

#include <algorithm>
#include <array>
#include <math.h>
#include <tuple>

using namespace std;
using namespace Geometry2d;

KickEvaluator::KickEvaluator(SystemState* systemState)
    : system(systemState) {}

double KickEvaluator::eval_pt_to_pt(Point origin,
                                      Point target,
                                      float targetWidth) {

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

    // < Dist, Angle >
    vector< tuple<double, double> > bot_locations;
    for_each(bots.begin(), bots.end(), [&bot_locations, target, origin](Robot* bot) {

        Point bot_dir = bot->pos - origin;
        double dist = bot_dir.mag();
        double angl = bot_dir.angleBetween(target - origin);

        // Only count bots in front of kicker
        if (fabs(angl) < M_PI_2) {
            bot_locations.push_back(make_tuple(dist, angl));
        }
    });

    // No bots in the way
    if (bot_locations.size() == 0) {
        return 1;
    }

    double half_target_width = targetWidth / 2;
    int num_rays = 16;
    double min_ray_weight = .1;

    double total = 0.0;
    double max_total = 0.0;

    double cur_ray_angle = -1 * half_target_width;
    double ray_angle_inc = targetWidth / num_rays;

    double distance_coefficient = -1.0 / 7.0;
    double kernal_width_coefficient = 32;

    while (cur_ray_angle < half_target_width ||
           nearlyEqual((float)cur_ray_angle, half_target_width)) {

        vector<double> scores;

        for (tuple<double, double> bot_location : bot_locations) {
            double angle_off_ray = get<1>(bot_location) - cur_ray_angle;

            // Distance from ray
            double u = sin(angle_off_ray) * get<0>(bot_location);
            // Overal kernal scale
            u *= kernal_width_coefficient;
            // Distance from kick point
            u *= exp(get<0>(bot_location) * kernal_width_coefficient);

            u = min(1.0, u);
            u = max(-1.0, u);

            scores.push_back(1 - max(pow((1 - pow(u, 2)), 3), 0.0));
        }

        double ray_offset_scale = 1 - (fabs(cur_ray_angle) / ((1+min_ray_weight) * half_target_width));

        double min_score = *min_element(begin(scores), end(scores));

        total += min_score * ray_offset_scale;
        max_total += ray_offset_scale;
        cur_ray_angle += ray_angle_inc;
    }

    return total / max_total;
}