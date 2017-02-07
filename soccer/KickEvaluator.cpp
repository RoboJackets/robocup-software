#include "KickEvaluator.hpp"
#include <Geometry2d/Util.hpp>

#include <algorithm>
#include <array>
#include <math.h>
#include <cmath>
#include <tuple>

REGISTER_CONFIGURABLE(KickEvaluator)

// 1 / sqrt(2*pi)
#define M_1_SQRT_2_PI 0.3989422

using namespace std;
using namespace Geometry2d;

ConfigDouble* KickEvaluator::robot_angle_filter_limit;
ConfigDouble* KickEvaluator::kick_std_dev;
ConfigDouble* KickEvaluator::num_rays;

ConfigDouble* KickEvaluator::kernal_width_coefficient;
ConfigDouble* KickEvaluator::distance_coefficient;

void KickEvaluator::createConfiguration(Configuration* cfg) {
    robot_angle_filter_limit = 
        new ConfigDouble(cfg, "KickEvaluator/robot_angle_filter_limit", 0.35 * M_PI);
    kick_std_dev = 
        new ConfigDouble(cfg, "KickEvaluator/kick_std_dev", 0.2 * M_PI);
    num_rays = 
        new ConfigDouble(cfg, "KickEvaluator/num_rays", 16);

    kernal_width_coefficient = 
        new ConfigDouble(cfg, "KickEvaluator/kernal_width_coefficient", 32.0);
    distance_coefficient = 
        new ConfigDouble(cfg, "KickEvaluator/distance_coefficient", -1.0 / 7.0);
}

KickEvaluator::KickEvaluator(SystemState* systemState)
    : system(systemState) {}

float KickEvaluator::eval_pt_to_pt(Point origin,
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
    vector< tuple<float, float> > bot_locations;

    // Add robots as obstacles
    for_each(bots.begin(), bots.end(), 
            [&bot_locations, target, origin, this](Robot* bot) {

        tuple<float, float> polar_coords = rect_to_polar(origin, target, bot->pos);

        if (fabs(get<1>(polar_coords)) < max_delta_angle) {
            bot_locations.push_back(polar_coords);
        }
    });

    // Add imaginary obstacles
    for_each(hypothetical_robot_locations.begin(), hypothetical_robot_locations.end(),
             [&bot_locations, target, origin, this](Point obstacle) {

        tuple<float, float> polar_coords = rect_to_polar(origin, target, obstacle);

        if (fabs(get<1>(polar_coords)) < max_delta_angle) {
            bot_locations.push_back(polar_coords);
        }
    });

    // Use rays between -3 * std_dev and 3 * std_dev
    float half_std_dev = 1.5f * *kick_std_dev;
    float half_target_width = fabs(atan2(targetWidth / 2, (target - origin).mag()));

    // No bots in the way
    if (bot_locations.size() == 0) {
        // CDF Estimation
        // The ray estimations are linearlly related to the true CDF probability
        // These are found through testing the points and using the best fit line
        // with R^2 = 0.998458
        return 1.1219 * erf(half_target_width / (*kick_std_dev * sqrt(2))) + 0.0125;
    }

    float total = 0.0f;
    float max_total = 0.0f;

    float cur_ray_angle = -1 * half_std_dev;
    float ray_angle_inc = half_std_dev / number_of_rays;

    // For each ray
    while (cur_ray_angle < half_std_dev ||
           nearlyEqual(cur_ray_angle, half_std_dev)) {

        vector<float> scores;

        // For each robot
        for (tuple<float, float> bot_location : bot_locations) {
            float angle_off_ray = get<1>(bot_location) - cur_ray_angle;

            // Distance from ray
            float u = sin(angle_off_ray) * get<0>(bot_location);
            // Overal kernal scale
            u *= *kernal_width_coefficient;
            // Distance from kick point
            u *= exp(get<0>(bot_location) * *distance_coefficient);

            u = min(1.0f, u);
            u = max(-1.0f, u);

            // Triweight kernal function
            scores.push_back(1 - (float)max(pow((1 - pow(u, 2)), 3), 0.0));
        }

        // Gets -1, 0, 1 depending on whether abs(cur_ray_angle) is > half_target_width
        float in_range = copysign(1.0, half_target_width - fabs(cur_ray_angle));
        // Moves in_range to 0 if outside, 1 if inside the range, 0.5 if on the edge
        in_range = in_range * 0.5 + 0.5;

        // PDF for Gaussian Distribution, Assume mean = 0
        float ray_offset_scale = M_1_SQRT_2_PI / *kick_std_dev;
        ray_offset_scale *= fast_exp(-0.5 * pow(cur_ray_angle / *kick_std_dev, 2));

        float min_score = *min_element(begin(scores), end(scores));

        // Only add to the total if it's within the target range
        total += in_range * min_score * ray_offset_scale;

        max_total += ray_offset_scale;
        cur_ray_angle += ray_angle_inc;
    }

    return total / max_total;
}

float KickEvaluator::eval_pt_to_robot(Geometry2d::Point origin,
                                      Geometry2d::Point target) {
    return eval_pt_to_pt(origin, target, 2 * Robot_Radius);
}

float KickEvaluator::eval_pt_to_opp_goal(Geometry2d::Point origin) {
    Segment their_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()}
    };

    return eval_pt_to_seg(origin, their_goal);
}

float KickEvaluator::eval_pt_to_our_goal(Geometry2d::Point origin) {
    Segment our_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0}
    };

    return eval_pt_to_seg(origin, our_goal);
}

float KickEvaluator::eval_pt_to_seg(Geometry2d::Point origin,
                                    Geometry2d::Segment target) {
    double angle = abs(atan2(target.pt[0].y(), target.pt[0].x()) - 
                       atan2(target.pt[1].y(), target.pt[0].x()));
    Point center = target.center();
    double target_width = center.mag() * sin(angle / 2) * 2;

    return eval_pt_to_pt(origin, center, target_width);
}

tuple<float, float> KickEvaluator::rect_to_polar(Point origin,
                                                 Point target,
                                                 Point obstacle) {
    Point dir = obstacle - origin;

    return make_tuple(dir.mag(), dir.angleBetween(target - origin));
}

float KickEvaluator::fast_exp(float x)
{
    return (24+x*(24+x*(12+x*(4+x))))*0.041666666f;
}