#include "KickEvaluator.hpp"
#include <Utils.hpp>
#include <Geometry2d/Util.hpp>

#include <algorithm>
#include <vector>
#include <math.h>
#include <cmath>

static union 
{
  float d;
  struct {
#ifdef LITTLE_ENDIAN
    int j,i;
#else 
    int i,j;
#endif
  } n;
} _eco;

// Floating point magic
#define EXP_A (1048576/0.69314718055994530942)
#define EXP_C 60801
#define fast_exp(y) (_eco.n.i = EXP_A*(y) + (1072693248 - EXP_C), _eco.d)

REGISTER_CONFIGURABLE(KickEvaluator)

using namespace std;
using namespace Geometry2d;

ConfigDouble* KickEvaluator::kick_std_dev;
ConfigDouble* KickEvaluator::kick_mean;
ConfigDouble* KickEvaluator::robot_std_dev;
ConfigDouble* KickEvaluator::start_x_offset;

void KickEvaluator::createConfiguration(Configuration* cfg) {
    kick_std_dev     = new ConfigDouble(cfg,
                   "KickEvaluator/kick_std_dev", 0.08);
    kick_mean        = new ConfigDouble(cfg,
                   "KickEvaluator/kick_mean", 0);
    robot_std_dev    = new ConfigDouble(cfg,
                   "KickEvaluator/robot_std_dev", 0.3);
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
    float targetWidth = get_target_angle(origin, target);

    // Polar bot locations
    // <Dist, Angle>
    vector< tuple<float, float> > botLocations = 
                            convert_robots_to_polar(origin, center);

    // Convert polar to mean / std_dev / Vertical Scales
    vector<float> botMeans;
    vector<float> botStDevs;
    vector<float> botVertScales;

    botMeans.reserve(botLocations.size());
    botStDevs.reserve(botLocations.size());
    botVertScales.reserve(botLocations.size());

    float distPastTarget;

    for (tuple<float, float>& loc : botLocations){
        botMeans.push_back(get<1>(loc));
        // Want std_dev in radians, not XY distance
        botStDevs.push_back(atan(*robot_std_dev / get<0>(loc)));

        // Robot Past Target
        distPastTarget = get<0>(loc) - (origin - center).mag();

        // If robot is past target, only use the chance at the target segment
        if (distPastTarget > 0 && fabs(get<1>(loc)) < M_PI / 2) {
            // Evaluate a normal distribution at dist away and scale
            float stdev2 = pow(*robot_std_dev, 2);
            botVertScales.push_back(1 / stdev2 * fast_exp(-0.5 * pow(distPastTarget, 2) / stdev2));
        } else {
            botVertScales.push_back(1);
        }
    }

    // No opponent robots on the field
    if (botMeans.size() == 0) {
        botMeans.push_back(10);
        // Must be non-zero as 1 / botStDev is used
        botStDevs.push_back(0.001);
        botVertScales.push_back(0);
    }

    // Create KickEvaluator Function parameters
    unique_ptr<KickEvaluatorArgs> keArgs( 
                    new KickEvaluatorArgs(*kick_mean, *kick_std_dev,
                                      botMeans, botStDevs, botVertScales,
                                      targetWidth / -2, targetWidth / 2));

    ParallelGradient1DConfig parallelConfig;
    init_gradient_configs(&parallelConfig, keArgs.get());

    // Create Gradient Ascent Optimizer and run it
    ParallelGradientAscent1D optimizer(&parallelConfig);

    optimizer.execute();

    // Grab the lcoal max values and their X location
    vector<float> maxXValues = optimizer.getMaxXValues();
    vector<float> maxValues = optimizer.getMaxValues();

    // Default to a local max
    int index = distance(maxValues.begin(), max_element(maxValues.begin(), maxValues.end()));
    float maxX = maxXValues.at(index);
    float maxChance = maxValues.at(index);

    // See if there is a segment which is longer
    // Since local maxes stop on either side of the segment
    if (maxXValues.size() > 1) {
        for (int i = 0; i < maxXValues.size() - 1; i++) {
            // Finds the score at the average between two local maxes
            float midPoint = (maxXValues.at(i) + maxXValues.at(i + 1)) / 2;
            float chance = get<0>(eval_calculation(midPoint, keArgs.get()));

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

tuple<float, float> KickEvaluator::eval_calculation(float x, FunctionArgs* fArgs) {
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

    // Note: Static is used as a time saver as this executes a few hundred times
    // Estimated 5-10% reduction

    // We want the worst chance of success
    static float minResults;
    static int minIndex;
    minResults = 1.0f;
    minIndex = 0;

    // Shortcuts for repeated operations
    static float kmean;
    static float kstdev;
    static float kstdev2;
    kmean = args->kickMean;
    kstdev = args->kickStDev;
    kstdev2 = kstdev * kstdev;

    static float sqrt2pi = 2.50662827f; // sqrt(2*PI)
    static float sqrtpi_2 = 1.253314137f; // sqrt(PI/2)
    static float sqrt2_kstdev;
    static float sqrt1_kstdev2;
    sqrt2_kstdev = (1.4142f * kstdev); // sqrt(2) * kstdev
    sqrt1_kstdev2 = 1.0f / sqrt(1.0f / kstdev2);

    static float rmean;
    static float rstdev;
    static float rstdev2;
    static float robotV;
    
    static float kx;
    kx = kmean - x;

    static float fterm; // First term,  Robot normal distribution
    static float sterm; // Second term, Left boundary
    static float tterm; // Third term,  Right Boundary

    static float results;
    static float derivative;

    sterm = sqrtpi_2 * (sqrt1_kstdev2 - kstdev * erf((kx + args->boundaryLower) / sqrt2_kstdev));

    tterm = sqrtpi_2 * (sqrt1_kstdev2 - kstdev * erf((kx + args->boundaryUpper) / sqrt2_kstdev));

    // For each robot distribution in Set #1
    static int i;
    for (i = 0; i < args->robotMeans.size(); i++) {
        rmean   = args->robotMeans[i];
        rstdev  = args->robotStDevs[i];
        rstdev2 = rstdev*rstdev;
        robotV  = args->robotVertScales[i];

        fterm = -1.0f * fast_exp(-0.5f * (kx + rmean)*(kx + rmean) / (kstdev2 + rstdev2)) * robotV * sqrt2pi;
        fterm = fterm / sqrt(1.0f / kstdev2 + 1.0f / rstdev2);

        results = 1.0f / (kstdev * sqrt2pi) * (fterm + sterm - tterm);

        if (results < minResults) {
            minResults = results;
            minIndex = i;
        }
    }

    // Calculate derivative of the convolution
    rmean   = args->robotMeans[minIndex];
    rstdev  = args->robotStDevs[minIndex];
    rstdev2 = rstdev*rstdev;
    robotV  = args->robotVertScales[minIndex];

    fterm = fast_exp(-0.5f * (kx + rmean)*(kx + rmean) / (kstdev2 + rstdev2)) * robotV * sqrt2pi * (kx + rmean);
    fterm = fterm / (sqrt(1.0f / kstdev2 + 1.0f / rstdev2) * (kstdev2 + rstdev2));

    sterm = fast_exp(-0.5f * (kx + args->boundaryLower)*(kx + args->boundaryLower) / kstdev2);

    tterm = fast_exp(-0.5f * (kx + args->boundaryUpper)*(kx + args->boundaryUpper) / kstdev2);

    derivative = 1.0f / (kstdev * sqrt2pi) * (sterm - tterm - fterm);

    return forward_as_tuple(minResults, derivative);
}


float KickEvaluator::get_target_angle(Point origin, Segment target) {
    Point left = target.pt[0] - origin;
    Point right = target.pt[1] - origin;

    return abs(left.angle() - right.angle());
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

tuple<float, float> KickEvaluator::rect_to_polar(Point origin,
                                                   Point target,
                                                   Point obstacle) {
    Point obstacleDir = obstacle - origin;
    Point targetDir = target - origin;
    float angle =  obstacleDir.angle() - targetDir.angle();

    return forward_as_tuple(obstacleDir.mag(), fixAngleRadians(angle));
}

vector< tuple<float, float> > KickEvaluator::convert_robots_to_polar(Point origin, Point target) {
    vector<Robot*> bots = get_valid_robots();
    vector< tuple<float, float> > botLocations;
    botLocations.reserve(bots.size() + botLocations.size());

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

void KickEvaluator::init_gradient_configs(ParallelGradient1DConfig* pConfig, KickEvaluatorArgs* keArgs) {
    pConfig->GA1DConfig.reserve(keArgs->robotStDevs.size());

    // Standard Gradient Configs
    float dxError = 0.05;
    float maxXMovement = *min_element(keArgs->robotStDevs.begin(), keArgs->robotStDevs.end()) * 2;
    float temperatureDescent = 0.5;
    float temperatureMin = 0.01;
    int maxIterations = 20;
    float maxValue = 1;
    float maxThresh = 0.05;
    float boundaryLower = keArgs->boundaryLower;
    float boundaryUpper = keArgs->boundaryUpper;

    // PrevStart, Start
    vector< tuple<float, float> > xStarts;
    xStarts.reserve(keArgs->robotStDevs.size());

    float startX = boundaryLower + *start_x_offset * keArgs->kickStDev;
    xStarts.push_back(forward_as_tuple(boundaryLower, startX));

    startX = boundaryUpper - *start_x_offset * keArgs->kickStDev;
    xStarts.push_back(forward_as_tuple(boundaryUpper, startX));

    // For each robot
    for (int i = 0; i < keArgs->robotMeans.size(); i++) {
        // -1 or 1
        for (int side = -1; side <= 1; side += 2) {
            startX = keArgs->robotMeans.at(i) + side * *start_x_offset * keArgs->robotStDevs.at(i);

            xStarts.push_back(forward_as_tuple(keArgs->robotMeans.at(i), startX));
        }
    }

    // Force into ascending order to make things simpler later on
    sort(xStarts.begin(), xStarts.end(), [&](tuple<float, float> a, tuple<float, float> b) {
        return get<1>(a) < get<1>(b);
    });

    // Create list of configs
    for (tuple<float, float> xStart : xStarts) {
        pConfig->GA1DConfig.emplace_back(&eval_calculation, keArgs,
                             get<1>(xStart), get<0>(xStart),
                             dxError, maxXMovement,
                             temperatureDescent, temperatureMin,
                             maxIterations, maxValue, maxThresh);
    
    }

    pConfig->xCombineThresh = *min_element(keArgs->robotStDevs.begin(), keArgs->robotStDevs.end()) * *start_x_offset / 2;
}