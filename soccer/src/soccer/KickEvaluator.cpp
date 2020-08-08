#include "KickEvaluator.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Geometry2d/Util.hpp>
#include <rj_common/Utils.hpp>

REGISTER_CONFIGURABLE(KickEvaluator)

using Geometry2d::Point, Geometry2d::Segment, Geometry2d::Line;
using std::tuple, std::vector, std::abs, std::make_tuple, std::function, std::pair, std::get;

ConfigDouble* KickEvaluator::kick_std_dev;
ConfigDouble* KickEvaluator::kick_mean;
ConfigDouble* KickEvaluator::robot_std_dev;
ConfigDouble* KickEvaluator::start_x_offset;

// Fast exp function, valid within 4% at +- 100
inline double fast_exp(double x) {
    double d;                                             // NOLINT
    *((int*)(&d) + 0) = 0;                                // NOLINT
    *((int*)(&d) + 1) = (int)(1512775 * x + 1072632447);  // NOLINT
    return d;
}

inline float fast_exp(float x) { return static_cast<float>(fast_exp(static_cast<double>(x))); }

void KickEvaluator::create_configuration(Configuration* cfg) {
    kick_std_dev = new ConfigDouble(cfg, "KickEvaluator/kick_std_dev", 0.04);
    kick_mean = new ConfigDouble(cfg, "KickEvaluator/kick_mean", 0);
    robot_std_dev = new ConfigDouble(cfg, "KickEvaluator/robot_std_dev", 0.3);
    start_x_offset = new ConfigDouble(cfg, "KickEvaluator/start_x_offset", 0.1);
}

KickEvaluator::KickEvaluator(SystemState* system_state) : system_(system_state) {}

KickResults KickEvaluator::eval_pt_to_pt(Point origin, Point target, float target_width) {
    Point dir = (target - origin).perp_ccw().normalized();
    Segment seg = Segment{target + dir * (target_width / 2), target - dir * (target_width / 2)};

    return eval_pt_to_seg(origin, seg);
}

KickResults KickEvaluator::eval_pt_to_robot(Point origin, Point target) {
    return eval_pt_to_pt(origin, target, 2 * kRobotRadius);
}

KickResults KickEvaluator::eval_pt_to_opp_goal(Point origin) {
    Segment their_goal{Point{-FieldDimensions::current_dimensions.goal_width() / 2,
                             FieldDimensions::current_dimensions.length()},
                       Point{FieldDimensions::current_dimensions.goal_width() / 2,
                             FieldDimensions::current_dimensions.length()}};

    return eval_pt_to_seg(origin, their_goal);
}

KickResults KickEvaluator::eval_pt_to_our_goal(Point origin) {
    Segment our_goal{Point{-FieldDimensions::current_dimensions.goal_width() / 2, 0},
                     Point{FieldDimensions::current_dimensions.goal_width() / 2, 0}};

    return eval_pt_to_seg(origin, our_goal);
}

KickResults KickEvaluator::eval_pt_to_seg(Point origin, Segment target) {
    Point center = target.center();
    float target_width = get_target_angle(origin, target);

    // Polar bot locations
    // <Dist, Angle>
    vector<tuple<float, float> > bot_locations = convert_robots_to_polar(origin, center);

    // Convert polar to mean / std_dev / Vertical Scales
    vector<float> bot_means;
    vector<float> bot_st_devs;
    vector<float> bot_vert_scales;

    bot_means.reserve(bot_locations.size());
    bot_st_devs.reserve(bot_locations.size());
    bot_vert_scales.reserve(bot_locations.size());

    float dist_past_target{};

    for (tuple<float, float>& loc : bot_locations) {
        bot_means.push_back(get<1>(loc));
        // Want std_dev in radians, not XY distance
        bot_st_devs.push_back(atan(*robot_std_dev / get<0>(loc)));

        // Robot Past Target
        dist_past_target = static_cast<float>(get<0>(loc) - (origin - center).mag());

        // If robot is past target, only use the chance at the target segment
        if (dist_past_target > 0 && fabs(get<1>(loc)) < M_PI / 2) {
            // Evaluate a normal distribution at dist away and scale
            bot_vert_scales.push_back(1 - erf(dist_past_target / (*robot_std_dev * sqrt(2))));
        } else {
            bot_vert_scales.push_back(1);
        }
    }

    // Create function with only 1 input
    // Rest are bound to constant values
    function<tuple<float, float>(float)> ke_func =
        bind(&eval_calculation, std::placeholders::_1, (kick_mean->value()),
             (kick_std_dev->value()), cref(bot_means), cref(bot_st_devs), cref(bot_vert_scales),
             target_width / -2, target_width / 2);

    // No opponent robots on the field
    if (bot_means.empty()) {
        // Push it off to the side
        bot_means.push_back(4);
        // Must be non-zero as 1 / bot_st_dev is used
        bot_st_devs.push_back(0.1);
        bot_vert_scales.push_back(0.001);

        // Center will always be the best target X with no robots
        return pair<Point, float>(center, get<0>(ke_func(0)));
    }

    ParallelGradient1DConfig parallel_config;
    KickEvaluator::init_gradient_configs(parallel_config, ke_func, bot_means, bot_st_devs,
                                         target_width / -2, target_width / 2);

    // Create Gradient Ascent Optimizer and run it
    ParallelGradientAscent1D optimizer(&parallel_config);

    optimizer.execute();

    // Grab the lcoal max values and their X location
    vector<float> max_x_values = optimizer.get_max_x_values();
    vector<float> max_values = optimizer.get_max_values();

    // Default to a local max
    int index = distance(max_values.begin(), max_element(max_values.begin(), max_values.end()));
    float max_x = max_x_values.at(index);
    float max_chance = max_values.at(index);

    // See if there is a segment which is longer
    // Since local maxes stop on either side of the segment
    if (max_x_values.size() > 1) {
        for (int i = 0; i < max_x_values.size() - 1; i++) {
            // Finds the score at the average between two local maxes
            float mid_point = (max_x_values.at(i) + max_x_values.at(i + 1)) / 2;
            float chance = get<0>(ke_func(mid_point));

            // chance >= max_chance
            if (chance > max_chance || nearly_equal(chance, max_chance)) {
                max_x = mid_point;
                max_chance = chance;
            }
        }
    }

    // Angle in reference to the field
    float real_max_angle = static_cast<float>(max_x + (center - origin).angle());
    Line best_kick_line(origin, origin + Point{cos(real_max_angle), sin(real_max_angle)});

    // Return point on target segment and chance
    return pair<Point, float>(target.nearest_point(best_kick_line), max_chance);
}

tuple<float, float> KickEvaluator::eval_calculation(float x, float kmean, float kstdev,
                                                    const vector<float>& robot_means,
                                                    const vector<float>& robot_st_devs,
                                                    const vector<float>& robot_vert_scales,
                                                    float b_left, float b_right) {
    // 3 Main distribution sets
    // Set #1 : A set of each normal distribution for the obstacles
    // Set #2 : A band pass style distribution that represents a valid target
    // kick
    // Set #3 : A normal distribution representing a kick

    // Set #1 and #2 are combined. To keep the convolution simple, Set #2 is
    // represented using
    //      a Unit step function and just added/subtracted to/from Set #1
    // This will cause problems along the edges when a robot is near since it
    // will go negative
    // But it is not /super/ significant

    // The resulting F(X) represents the convolution of Set #12 and Set #3
    // All of this is calculated with Mathematica

    // We want the worst chance of success
    float min_results = 1.0f;
    int min_index = 0;

    // Shortcuts for repeated operations
    float kstdev2 = kstdev * kstdev;

    const float sqrt2pi = 2.50662827f;        // sqrt(2*PI)
    const float sqrtpi_2 = 1.253314137f;      // sqrt(PI/2)
    float sqrt2_kstdev = (1.4142f * kstdev);  // sqrt(2) * kstdev
    float sqrt1_kstdev2 = 1.0f / sqrt(1.0f / kstdev2);

    float rmean{};
    float rstdev{};
    float rstdev2{};
    float robot_v{};

    float kx = kmean - x;

    float fterm{};  // First term,  Robot normal distribution
    float sterm{};  // Second term, Left boundary
    float tterm{};  // Third term,  Right Boundary

    float results{};
    float derivative{};

    sterm = sqrtpi_2 * (sqrt1_kstdev2 - kstdev * erf((kx + b_left) / sqrt2_kstdev));

    tterm = sqrtpi_2 * (sqrt1_kstdev2 - kstdev * erf((kx + b_right) / sqrt2_kstdev));

    // For each robot distribution in Set #1
    for (int i = 0; i < robot_means.size(); i++) {
        rmean = robot_means[i];
        rstdev = robot_st_devs[i];
        rstdev2 = rstdev * rstdev;
        robot_v = robot_vert_scales[i];

        fterm = -1.0f * fast_exp(-0.5f * (kx + rmean) * (kx + rmean) / (kstdev2 + rstdev2)) *
                robot_v * sqrt2pi;
        fterm = fterm / sqrt(1.0f / kstdev2 + 1.0f / rstdev2);

        results = 1.0f / (kstdev * sqrt2pi) * (fterm + sterm - tterm);

        if (results < min_results) {
            min_results = results;
            min_index = i;
        }
    }

    // Calculate derivative of the convolution
    rmean = robot_means[min_index];
    rstdev = robot_st_devs[min_index];
    rstdev2 = rstdev * rstdev;
    robot_v = robot_vert_scales[min_index];

    fterm = fast_exp(-0.5f * (kx + rmean) * (kx + rmean) / (kstdev2 + rstdev2)) * robot_v *
            sqrt2pi * (kx + rmean);
    fterm = fterm / (sqrt(1.0f / kstdev2 + 1.0f / rstdev2) * (kstdev2 + rstdev2));

    sterm = fast_exp(-0.5f * (kx + b_left) * (kx + b_left) / kstdev2);

    tterm = fast_exp(-0.5f * (kx + b_right) * (kx + b_right) / kstdev2);

    derivative = 1.0f / (kstdev * sqrt2pi) * (sterm - tterm - fterm);

    return make_tuple(min_results, derivative);
}

float KickEvaluator::get_target_angle(Point origin, Segment target) {
    Point left = target.pt[0] - origin;
    Point right = target.pt[1] - origin;

    return static_cast<float>(abs(left.angle_between(right)));
}

vector<Robot*> KickEvaluator::get_valid_robots() {
    vector<Robot*> bots(system_->self.size() + system_->opp.size());

    auto filter_predicate = [&](const Robot* bot) -> bool {
        return bot != nullptr && bot->visible() &&
               find(excluded_robots.begin(), excluded_robots.end(), bot) == excluded_robots.end();
    };

    auto end_it =
        std::copy_if(system_->self.begin(), system_->self.end(), bots.begin(), filter_predicate);

    end_it = std::copy_if(system_->opp.begin(), system_->opp.end(), end_it, filter_predicate);

    bots.resize(distance(bots.begin(), end_it));

    return bots;
}

tuple<float, float> KickEvaluator::rect_to_polar(Point origin, Point target, Point obstacle) {
    Point obstacle_dir = obstacle - origin;
    Point target_dir = target - origin;

    return make_tuple(obstacle_dir.mag(),
                      fix_angle_radians(target_dir.angle_between(obstacle_dir)));
}

vector<tuple<float, float> > KickEvaluator::convert_robots_to_polar(Point origin, Point target) {
    vector<Robot*> bots = get_valid_robots();
    vector<tuple<float, float> > bot_locations;
    bot_locations.reserve(bots.size() + bot_locations.size());

    // Convert each bot position to polar
    transform(
        bots.begin(), bots.end(), back_inserter(bot_locations),
        [target, origin, this](Robot* bot) { return rect_to_polar(origin, target, bot->pos()); });

    // Convert imaginary obstacles to polar
    transform(hypothetical_robot_locations.begin(), hypothetical_robot_locations.end(),
              back_inserter(bot_locations), [target, origin, this](Point obstacle) {
                  return rect_to_polar(origin, target, obstacle);
              });

    return bot_locations;
}

void KickEvaluator::init_gradient_configs(ParallelGradient1DConfig& p_config,
                                          function<tuple<float, float>(float)>& func,
                                          const vector<float>& robot_means,
                                          const vector<float>& robot_st_devs, float boundary_lower,
                                          float boundary_upper) {
    p_config.ga_config.reserve(robot_st_devs.size());

    // Standard Gradient Configs
    const float dx_error = 0.05;
    float max_x_movement = *min_element(robot_st_devs.begin(), robot_st_devs.end()) * 2;
    const float temperature_descent = 0.5;
    const float temperature_min = 0.01;
    const int max_iterations = 20;
    const float max_value = 1;
    const float max_thresh = 0.05;

    // <PrevStart, Start>
    vector<tuple<float, float> > x_starts;
    x_starts.reserve(robot_st_devs.size());

    // Add left boundary
    auto start_x = static_cast<float>(boundary_lower + *start_x_offset * *kick_std_dev);
    x_starts.emplace_back(boundary_lower, start_x);

    // Add right boundary
    start_x = boundary_upper - *start_x_offset * *kick_std_dev;
    x_starts.emplace_back(boundary_upper, start_x);

    // For each robot
    for (int i = 0; i < robot_means.size(); i++) {
        // -1 or 1
        for (int side = -1; side <= 1; side += 2) {
            start_x = robot_means.at(i) + side * *start_x_offset * robot_st_devs.at(i);

            x_starts.emplace_back(robot_means.at(i), start_x);
        }
    }

    // Force into ascending order to make things simpler later on
    sort(x_starts.begin(), x_starts.end(),
         [&](tuple<float, float> a, tuple<float, float> b) { return get<1>(a) < get<1>(b); });

    // Create list of configs
    for (tuple<float, float> x_start : x_starts) {
        p_config.ga_config.emplace_back(&func, get<1>(x_start), get<0>(x_start), dx_error,
                                        max_x_movement, temperature_descent, temperature_min,
                                        max_iterations, max_value, max_thresh);
    }

    p_config.x_combine_thresh = static_cast<float>(
        *min_element(robot_st_devs.begin(), robot_st_devs.end()) * *start_x_offset / 2);
}
