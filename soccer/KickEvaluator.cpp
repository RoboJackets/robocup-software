#include "KickEvaluator.hpp"
#include <Utils.hpp>
#include <Geometry2d/Util.hpp>

#include <algorithm>
#include <vector>
#include <math.h>
#include <cmath>

REGISTER_CONFIGURABLE(KickEvaluator)

using namespace std;
using namespace Geometry2d;

ConfigDouble* KickEvaluator::kick_std_dev;
ConfigDouble* KickEvaluator::kick_mean;
ConfigDouble* KickEvaluator::robot_std_dev;
ConfigDouble* KickEvaluator::start_x_offset;

// Fast exp function, valid within 4% at +- 100
inline double fast_exp(double x) {
    double d;
    *((int*)(&d) + 0) = 0;
    *((int*)(&d) + 1) = (int)(1512775 * x + 1072632447);
    return d;
}

void KickEvaluator::createConfiguration(Configuration* cfg) {
    kick_std_dev = new ConfigDouble(cfg, "KickEvaluator/kick_std_dev", 0.04);
    kick_mean = new ConfigDouble(cfg, "KickEvaluator/kick_mean", 0);
    robot_std_dev = new ConfigDouble(cfg, "KickEvaluator/robot_std_dev", 0.3);
    start_x_offset = new ConfigDouble(cfg, "KickEvaluator/start_x_offset", 0.1);
}

KickEvaluator::KickEvaluator(SystemState* systemState) : system(systemState) {}

KickResults KickEvaluator::eval_pt_to_pt(Point origin, Point target,
                                         float targetWidth) {
    Point dir = (target - origin).perpCCW().normalized();
    Segment seg = Segment{target + dir * (targetWidth / 2),
                          target - dir * (targetWidth / 2)};

    return eval_pt_to_seg(origin, seg);
}

KickResults KickEvaluator::eval_pt_to_robot(Point origin, Point target) {
    return eval_pt_to_pt(origin, target, 2 * Robot_Radius);
}

KickResults KickEvaluator::eval_pt_to_opp_goal(Point origin) {
    Segment their_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()}};

    return eval_pt_to_seg(origin, their_goal);
}

KickResults KickEvaluator::eval_pt_to_our_goal(Point origin) {
    Segment our_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0}};

    return eval_pt_to_seg(origin, our_goal);
}

KickResults KickEvaluator::eval_pt_to_seg(Point origin, Segment target) {
    Point center = target.center();
    float targetWidth = get_target_angle(origin, target);

    // Polar bot locations
    // <Dist, Angle>
    vector<tuple<float, float> > botLocations =
        convert_robots_to_polar(origin, center);

    // Convert polar to mean / std_dev / Vertical Scales
    vector<float> botMeans;
    vector<float> botStDevs;
    vector<float> botVertScales;

    botMeans.reserve(botLocations.size());
    botStDevs.reserve(botLocations.size());
    botVertScales.reserve(botLocations.size());

    float distPastTarget;

    for (tuple<float, float>& loc : botLocations) {
        botMeans.push_back(get<1>(loc));
        // Want std_dev in radians, not XY distance
        botStDevs.push_back(atan(*robot_std_dev / get<0>(loc)));

        // Robot Past Target
        distPastTarget = get<0>(loc) - (origin - center).mag();

        // If robot is past target, only use the chance at the target segment
        if (distPastTarget > 0 && fabs(get<1>(loc)) < M_PI / 2) {
            // Evaluate a normal distribution at dist away and scale
            float stdev2 = pow(*robot_std_dev, 2);
            botVertScales.push_back(
                fast_exp(-0.5 * pow(distPastTarget, 2) / stdev2));
        } else {
            botVertScales.push_back(1);
        }
    }

    // Create function with only 1 input
    // Rest are bound to constant values
    function<tuple<float, float>(float)> keFunc =
        bind(&eval_calculation, std::placeholders::_1, (*kick_mean),
             (*kick_std_dev), cref(botMeans), cref(botStDevs),
             cref(botVertScales), targetWidth / -2, targetWidth / 2);

    // No opponent robots on the field
    if (botMeans.size() == 0) {
        // Push it off to the side
        botMeans.push_back(4);
        // Must be non-zero as 1 / botStDev is used
        botStDevs.push_back(0.1);
        botVertScales.push_back(0.001);

        // Center will always be the best target X with no robots
        return pair<Point, float>(center, get<0>(keFunc(0)));
    }

    ParallelGradient1DConfig parallelConfig;
    init_gradient_configs(parallelConfig, keFunc, botMeans, botStDevs,
                          targetWidth / -2, targetWidth / 2);

    // Create Gradient Ascent Optimizer and run it
    ParallelGradientAscent1D optimizer(&parallelConfig);

    optimizer.execute();

    // Grab the lcoal max values and their X location
    vector<float> maxXValues = optimizer.getMaxXValues();
    vector<float> maxValues = optimizer.getMaxValues();

    // Default to a local max
    int index = distance(maxValues.begin(),
                         max_element(maxValues.begin(), maxValues.end()));
    float maxX = maxXValues.at(index);
    float maxChance = maxValues.at(index);

    // See if there is a segment which is longer
    // Since local maxes stop on either side of the segment
    if (maxXValues.size() > 1) {
        for (int i = 0; i < maxXValues.size() - 1; i++) {
            // Finds the score at the average between two local maxes
            float midPoint = (maxXValues.at(i) + maxXValues.at(i + 1)) / 2;
            float chance = get<0>(keFunc(midPoint));

            // chance >= maxChance
            if (chance > maxChance || nearlyEqual(chance, maxChance)) {
                maxX = midPoint;
                maxChance = chance;
            }
        }
    }

    // Angle in reference to the field
    float realMaxAngle = maxX + (center - origin).angle();
    Line bestKickLine(origin, Point{cos(realMaxAngle), sin(realMaxAngle)});

    // Return point on target segment and chance
    return pair<Point, float>(target.nearestPoint(bestKickLine), maxChance);
}

tuple<float, float> KickEvaluator::eval_calculation(
    const float x, const float kmean, const float kstdev,
    const vector<float>& robotMeans, const vector<float>& robotStDevs,
    const vector<float>& robotVertScales, const float bLeft,
    const float bRight) {
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
    float minResults = 1.0f;
    int minIndex = 0;

    // Shortcuts for repeated operations
    float kstdev2 = kstdev * kstdev;

    const float sqrt2pi = 2.50662827f;        // sqrt(2*PI)
    const float sqrtpi_2 = 1.253314137f;      // sqrt(PI/2)
    float sqrt2_kstdev = (1.4142f * kstdev);  // sqrt(2) * kstdev
    float sqrt1_kstdev2 = 1.0f / sqrt(1.0f / kstdev2);

    float rmean;
    float rstdev;
    float rstdev2;
    float robotV;

    float kx = kmean - x;

    float fterm;  // First term,  Robot normal distribution
    float sterm;  // Second term, Left boundary
    float tterm;  // Third term,  Right Boundary

    float results;
    float derivative;

    sterm =
        sqrtpi_2 * (sqrt1_kstdev2 - kstdev * erf((kx + bLeft) / sqrt2_kstdev));

    tterm =
        sqrtpi_2 * (sqrt1_kstdev2 - kstdev * erf((kx + bRight) / sqrt2_kstdev));

    // For each robot distribution in Set #1
    for (int i = 0; i < robotMeans.size(); i++) {
        rmean = robotMeans[i];
        rstdev = robotStDevs[i];
        rstdev2 = rstdev * rstdev;
        robotV = robotVertScales[i];

        fterm = -1.0f * fast_exp(-0.5f * (kx + rmean) * (kx + rmean) /
                                 (kstdev2 + rstdev2)) *
                robotV * sqrt2pi;
        fterm = fterm / sqrt(1.0f / kstdev2 + 1.0f / rstdev2);

        results = 1.0f / (kstdev * sqrt2pi) * (fterm + sterm - tterm);

        if (results < minResults) {
            minResults = results;
            minIndex = i;
        }
    }

    // Calculate derivative of the convolution
    rmean = robotMeans[minIndex];
    rstdev = robotStDevs[minIndex];
    rstdev2 = rstdev * rstdev;
    robotV = robotVertScales[minIndex];

    fterm =
        fast_exp(-0.5f * (kx + rmean) * (kx + rmean) / (kstdev2 + rstdev2)) *
        robotV * sqrt2pi * (kx + rmean);
    fterm =
        fterm / (sqrt(1.0f / kstdev2 + 1.0f / rstdev2) * (kstdev2 + rstdev2));

    sterm = fast_exp(-0.5f * (kx + bLeft) * (kx + bLeft) / kstdev2);

    tterm = fast_exp(-0.5f * (kx + bRight) * (kx + bRight) / kstdev2);

    derivative = 1.0f / (kstdev * sqrt2pi) * (sterm - tterm - fterm);

    return make_tuple(minResults, derivative);
}

float KickEvaluator::get_target_angle(const Point origin,
                                      const Segment target) {
    Point left = target.pt[0] - origin;
    Point right = target.pt[1] - origin;

    return abs(left.angleBetween(right));
}

vector<Robot*> KickEvaluator::get_valid_robots() {
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

    return bots;
}

tuple<float, float> KickEvaluator::rect_to_polar(const Point origin,
                                                 const Point target,
                                                 const Point obstacle) {
    Point obstacleDir = obstacle - origin;
    Point targetDir = target - origin;

    return make_tuple(obstacleDir.mag(),
                      fixAngleRadians(targetDir.angleBetween(obstacleDir)));
}

vector<tuple<float, float> > KickEvaluator::convert_robots_to_polar(
    const Point origin, const Point target) {
    vector<Robot*> bots = get_valid_robots();
    vector<tuple<float, float> > botLocations;
    botLocations.reserve(bots.size() + botLocations.size());

    // Convert each bot position to polar
    transform(bots.begin(), bots.end(), back_inserter(botLocations),
              [target, origin, this](Robot* bot) {
                  return rect_to_polar(origin, target, bot->pos);
              });

    // Convert imaginary obstacles to polar
    transform(hypothetical_robot_locations.begin(),
              hypothetical_robot_locations.end(), back_inserter(botLocations),
              [target, origin, this](Point obstacle) {
                  return rect_to_polar(origin, target, obstacle);
              });

    return botLocations;
}

void KickEvaluator::init_gradient_configs(
    ParallelGradient1DConfig& pConfig,
    function<tuple<float, float>(float)>& func, const vector<float>& robotMeans,
    const vector<float>& robotStDevs, const float boundaryLower,
    const float boundaryUpper) {
    pConfig.GA1DConfig.reserve(robotStDevs.size());

    // Standard Gradient Configs
    const float dxError = 0.05;
    float maxXMovement =
        *min_element(robotStDevs.begin(), robotStDevs.end()) * 2;
    const float temperatureDescent = 0.5;
    const float temperatureMin = 0.01;
    const int maxIterations = 20;
    const float maxValue = 1;
    const float maxThresh = 0.05;

    // <PrevStart, Start>
    vector<tuple<float, float> > xStarts;
    xStarts.reserve(robotStDevs.size());

    // Add left boundary
    float startX = boundaryLower + *start_x_offset * *kick_std_dev;
    xStarts.emplace_back(boundaryLower, startX);

    // Add right boundary
    startX = boundaryUpper - *start_x_offset * *kick_std_dev;
    xStarts.emplace_back(boundaryUpper, startX);

    // For each robot
    for (int i = 0; i < robotMeans.size(); i++) {
        // -1 or 1
        for (int side = -1; side <= 1; side += 2) {
            startX =
                robotMeans.at(i) + side * *start_x_offset * robotStDevs.at(i);

            xStarts.emplace_back(robotMeans.at(i), startX);
        }
    }

    // Force into ascending order to make things simpler later on
    sort(xStarts.begin(), xStarts.end(),
         [&](tuple<float, float> a, tuple<float, float> b) {
             return get<1>(a) < get<1>(b);
         });

    // Create list of configs
    for (tuple<float, float> xStart : xStarts) {
        pConfig.GA1DConfig.emplace_back(&func, get<1>(xStart), get<0>(xStart),
                                        dxError, maxXMovement,
                                        temperatureDescent, temperatureMin,
                                        maxIterations, maxValue, maxThresh);
    }

    pConfig.xCombineThresh =
        *min_element(robotStDevs.begin(), robotStDevs.end()) * *start_x_offset /
        2;
}