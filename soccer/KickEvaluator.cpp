#include "KickEvaluator.hpp"

#include <vector>
#include <math.h>
#include <cmath>

REGISTER_CONFIGURABLE(KickEvaluator)

using namespace std;
using namespace Geometry2d;

ConfigDouble* KickEvaluator::kick_std_dev;
ConfigDouble* KickEvaluator::kick_mean;
ConfigDouble* KickEvaluator::robot_std_dev;
ConfigDouble* KickEvaluator::robot_dist_scale;
ConfigDouble* KickEvaluator::start_x_offset;

void KickEvaluator::createConfiguration(Configuration* cfg) {
    kick_std_dev     = new ConfigDouble(cfg,
                   "KickEvaluator/kick_std_dev", 0.08);
    kick_mean        = new ConfigDouble(cfg,
                   "KickEvaluator/kick_mean", 0);
    robot_std_dev    = new ConfigDouble(cfg,
                   "KickEvaluator/robot_std_dev", 0.1);
    robot_dist_scale = new ConfigDouble(cfg,
                   "KickEvaluator/robot_dist_scale", 1);
    start_x_offset   = new ConfigDouble(cfg,
                   "KickEvaluator/start_x_offset", 0.1);
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
    return eval_pt_to_pt(origin, target, 2* Robot_Radius);
}

KickResults KickEvaluator::eval_pt_to_opp_goal(Point origin) {
    Segment their_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2,
              Field_Dimensions::Current_Dimensions.Length()}
    };

    return eval_pt_to_seg(origin, their_goal);
}

KickResults KickEvaluator::eval_pt_to_our_goal(Point origin) {
    Segment our_goal{
        Point{-Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0},
        Point{Field_Dimensions::Current_Dimensions.GoalWidth() / 2, 0}
    };

    return eval_pt_to_seg(origin, our_goal);
}

KickResults KickEvaluator::eval_pt_to_seg(Point origin, Segment target) {
    Point center = target.center();
    double targetWidth = get_reference_angle(origin, target);

    // Polar bot locations
    // <Dist, Angle>
    vector< tuple<double, double> > botLocations = 
                            convert_robots_to_polar(origin, center);

    // Convert polar to mean / std_dev / Vertical Scales
    vector<double> botMeans;
    vector<double> botStDevs;
    vector<double> botVertScales;

    for (tuple<double, double>& loc : botLocations){
        botMeans.push_back(get<1>(loc));
        // Want std_dev in radians, not XY distance
        botStDevs.push_back(acos(*robot_dist_scale / get<0>(loc)));

        // Robot Past Target
        double distPastTarget = get<0>(loc) - (origin - center).mag();

        // If robot is past target, only use the chance at the target segment
        if (distPastTarget > 0) {
            // Evaluate a normal distribution at dist away and scale
            double stdev2 = pow(*robot_dist_scale, 2);
            botVertScales.push_back(1 / stdev2 * exp(-0.5 * pow(distPastTarget, 2) / stdev2));
        } else {
            botVertScales.push_back(1);
        }
    }

    // Create KickEvaluator Function parameters
    unique_ptr<KickEvaluatorArgs> keArgs( 
                    new KickEvaluatorArgs(*kick_mean, *kick_std_dev,
                                      botMeans, botStDevs, botVertScales,
                                      targetWidth / -2, targetWidth / 2));

    ParallelGradient1DConfig parallelConfig = init_gradient_configs(keArgs.get());

    // Create Gradient Ascent Optimizer and run it
    ParallelGradientAscent1D optimizer(parallelConfig);

    optimizer.execute();

    // Grab the lcoal max values and their X location
    vector<double> maxXValues = optimizer.getMaxXValues();
    vector<double> maxValues = optimizer.getMaxValues();

    // Find the segment out of the list
    double maxX;
    double maxChance;

    // More than one local max
    if (maxXValues.size() > 1) {
        // This happens when there is a "flat" top
        // Find the highest "area" (RightX - LeftX) * MidPointChance
        double maxArea = 0;
        maxX = 0;
        maxChance = 0;

        for (int i = 0; i < maxXValues.size() - 1; i++) {
            double deltaX = maxXValues.at(i + 1) - maxXValues.at(i);
            double midPoint = (maxXValues.at(i + 1) - maxXValues.at(i)) / 2;
            double chance = get<0>(eval_calculation(midPoint, keArgs.get()));

            if (chance * deltaX > maxArea) {
                maxArea = chance * deltaX;
                maxX = midPoint;
                maxChance = chance;
            }
        }
    } else {
        maxX = maxXValues.at(0);
        maxChance = maxValues.at(0);
    }

    // Angle in reference to the field
    double realMaxAngle = maxX + get_reference_angle(origin, target);
    Line bestKickLine(origin, Point{cos(realMaxAngle), sin(realMaxAngle)});

    // Return point on target segment and chance
    return pair<Point, double>(target.nearestPoint(bestKickLine), maxChance);
}

tuple<double, double> KickEvaluator::eval_calculation(double x, FunctionArgs* fArgs) {
    // 3 Main distribution sets
    // Set #1 : A set of each normal distribution for the obstacles
    // Set #2 : A band pass style distribution that represents a valid target kick
    // Set #3 : A normal distribution representing a kick

    // Set #1 and #2 are combined. To keep the convolution simple, Set #2 is represented using
    //      a Unit step function and just added/subtracted to/from Set #1
    // This will cause problems along the edges when a robot is near since it will go negative
    // But it is not /super/ significant

    // The resulting F(X) represents the convolution of Set #12 and Set #3
    // All of this is calculated with Mathematica

    KickEvaluatorArgs* args = static_cast<KickEvaluatorArgs*>(fArgs);

    // We want the worst chance of success
    double minResults = 1;
    double minIndex = 0;


    // Shortcuts for repeated operations
    double sqrt2pi = sqrt(2*M_PI);
    double sqrtpi_2 = sqrt(M_PI / 2);

    double kmean = args->kickMean;
    double kstdev = args->kickStDev;
    double kstdev2 = pow(kstdev, 2);

    double rmean;
    double rstdev;
    double rstdev2;
    double robotV;
    
    double kx = kmean - x;

    double fterm; // First term,  Robot normal distribution
    double sterm; // Second term, Left boundary
    double tterm; // Third term,  Right Boundary

    // For each robot distribution in Set #1
    for (int i = 0; i < args->robotMeans.size(); i++) {
        rmean   = args->robotMeans.at(i);
        rstdev  = args->robotStDevs.at(i);
        rstdev2 = pow(rstdev, 2);
        robotV  = args->robotVertScales.at(i);

        fterm = -1 * exp(-0.5 * pow(kx + rmean, 2) / (kstdev2 + rstdev2)) * robotV * sqrt2pi;
        fterm /= sqrt(1 / kstdev2 + 1 / rstdev2);

        sterm = 1 / sqrt(1 / kstdev2) - kstdev * erf((kx + args->boundaryLower) / (sqrt(2) * kstdev));
        sterm *= sqrtpi_2;

        tterm = 1 / sqrt(1 / kstdev2) - kstdev * erf((kx + args->boundaryUpper) / (sqrt(2) * kstdev));
        sterm *= sqrtpi_2;

        double results = 1 / (kstdev * sqrt2pi) * (fterm + sterm - tterm);

        if (results < minResults) {
            minResults = results;
            minIndex = i;
        }
    }

    // Calculate derivative of the convolution
    rmean   = args->robotMeans.at(minIndex);
    rstdev  = args->robotStDevs.at(minIndex);
    rstdev2 = pow(rstdev, 2);
    robotV  = args->robotVertScales.at(minIndex);

    fterm = exp(-0.5 * pow(kx + rmean, 2) / (kstdev2 + rstdev2)) * robotV * sqrt2pi * (kx + rmean);
    fterm /= sqrt(1 / kstdev2 + 1 / rstdev2) * (kstdev2 + rstdev2);

    sterm = exp(-0.5 * pow(kx + args->boundaryLower, 2) / kstdev2);

    tterm = exp(-0.5 * pow(kx + args->boundaryUpper, 2) / kstdev2);

    double derivative = 1 / (kstdev * sqrt2pi) * (sterm - tterm - fterm);

    return make_tuple(minResults, derivative);
}


double KickEvaluator::get_reference_angle(Point origin, Segment target) {
    Point left = target.pt[0] - origin;
    Point right = target.pt[1] - origin;

    return abs(atan2(left.y(), left.x()) - atan2(right.y(), right.x()));
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

    end_it = copy_if(system->opp.begin(), system->opp.end(), 
                     end_it, filter_predicate);

    bots.resize(distance(bots.begin(), end_it));

    return bots;
}

tuple<double, double> KickEvaluator::rect_to_polar(Point origin,
                                                   Point target,
                                                   Point obstacle) {
    Point dir = obstacle - origin;

    return make_tuple(dir.mag(), dir.angleBetween(target - origin));
}

vector< tuple<double, double> > KickEvaluator::convert_robots_to_polar(Point origin, Point target) {
    vector<Robot*> bots = get_valid_robots();
    vector< tuple<double, double> > botLocations;

    // Convert each bot position to polar
    for_each(bots.begin(), bots.end(),
             [&botLocations, target, origin, this](Robot* bot) {
                botLocations.push_back(rect_to_polar(origin,
                                                      target, 
                                                      bot->pos));
             });

    // Convert imaginary obstacles to polar
    for_each(hypothetical_robot_locations.begin(),
             hypothetical_robot_locations.end(),
             [&botLocations, target, origin, this](Point obstacle) {
                botLocations.push_back(rect_to_polar(origin,
                                                      target, 
                                                      obstacle));
             });

    return botLocations;
}

ParallelGradient1DConfig KickEvaluator::init_gradient_configs(KickEvaluatorArgs* keArgs) {
    // Create list of single configs
    vector<Gradient1DConfig> singleGradientConfigs;

    // Standard Gradient Configs
    double dxError = 0.05;
    double maxXMovement = *min_element(keArgs->robotStDevs.begin(), keArgs->robotStDevs.end()) / 5;
    double temperatureDescent = 0.5;
    double temperatureMin = 0.01;
    int maxIterations = 100;
    double maxValue = 1;
    double maxThresh = 0.05;
    double boundaryLower = keArgs->boundaryLower;
    double boundaryUpper = keArgs->boundaryLower;

    // startX is ascending to make it easy later on
    double startX = boundaryLower + *start_x_offset * keArgs->kickStDev;

    singleGradientConfigs.push_back(
        Gradient1DConfig(&eval_calculation, keArgs,
                         startX, boundaryLower,
                         dxError, maxXMovement,
                         temperatureDescent, temperatureMin,
                         maxIterations, maxValue, maxThresh));

    // For each robot
    for (int i = 0; i < keArgs->robotMeans.size(); i++) {
        // -1 or 1
        for (int side = -1; side <= 1; side += 2) {
            startX = keArgs->robotMeans.at(i) + side * *start_x_offset * keArgs->robotStDevs.at(i);

            singleGradientConfigs.push_back(
                Gradient1DConfig(&eval_calculation, keArgs,
                                 startX, keArgs->robotMeans.at(i),
                                 dxError, maxXMovement,
                                 temperatureDescent, temperatureMin,
                                 maxIterations, maxValue, maxThresh));
        }
    }

    startX = boundaryUpper - *start_x_offset * keArgs->kickStDev;

    singleGradientConfigs.push_back(
        Gradient1DConfig(&eval_calculation, keArgs,
                         startX, boundaryUpper,
                         dxError, maxXMovement,
                         temperatureDescent, temperatureMin,
                         maxIterations, maxValue, maxThresh));

    // Create config from all the singles and the min std_dev
    double xCombineThresh = *min_element(keArgs->robotStDevs.begin(), keArgs->robotStDevs.end()) * *start_x_offset / 2;

    return ParallelGradient1DConfig(singleGradientConfigs, xCombineThresh);
}