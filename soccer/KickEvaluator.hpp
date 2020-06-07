#pragma once

#include <geometry2d/point.h>
#include <geometry2d/segment.h>

#include <functional>
#include <vector>

#include "Robot.hpp"
#include "SystemState.hpp"
#include "optimization/ParallelGradient1DConfig.hpp"
#include "optimization/ParallelGradientAscent1D.hpp"

// < [Point along target segment to aim at], [% Chance of success] >
using KickResults = std::pair<geometry2d::Point, float>;

/**
 * @brief Finds the best position to kick to and the chance of success
 */
class KickEvaluator {
public:
    /**
     * @brief Constructor
     * @param systemState, pointer to the global system state object
     */
    KickEvaluator(SystemState* systemState);

    /**
     * @brief Evaluates kick to target point
     * @param origin, Starting point of the kick
     * @param target, End point of the kick
     * @param targetWidth, Width of target in radians
     * @return Results of calculations
     */
    KickResults eval_pt_to_pt(geometry2d::Point origin,
                              geometry2d::Point target, float targetWidth);

    /**
     * @brief Evaluates kick to target robot
     * @param origin, Starting point of the kick
     * @param target, Target robot location
     * @return Results of calculations
     */
    KickResults eval_pt_to_robot(geometry2d::Point origin,
                                 geometry2d::Point target);

    /**
     * @brief Evaluates kick to opponent goal
     * @param origin, Starting point of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_opp_goal(geometry2d::Point origin);

    /**
     * @brief Evaluates kick to our goal
     * @param origin, Starting point of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_our_goal(geometry2d::Point origin);

    /**
     * @brief Evaluates kick to target segment
     * @param origin, Starting point of the kick
     * @param target, End segment of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_seg(geometry2d::Point origin,
                               geometry2d::Segment target);

    /**
     * @brief Evaluates closed form solution of the KickEvaluation problem
     * @param x, Location to run at
     * @param kmean, Kick mean
     * @param kstdev, Kick standard deviation
     * @param robotMeans, vector of robot angle locations
     * @param robotStDevs, vector of robot movement standard deviations
     * @param robotVertScales, vector of how much to scale the height
     * @param bLeft, left boundary angle
     * @param bRight, right boundary angle
     * @return F(X), F'(X)
     */
    static std::tuple<float, float> eval_calculation(
        float x, float kmean, float kstdev,
        const std::vector<float>& robotMeans,
        const std::vector<float>& robotStDevs,
        const std::vector<float>& robotVertScales, float bLeft, float bRight);

    /**
     * @brief Initializes configurable fields
     * @note See configuration documentation for details
     */
    static void createConfiguration(Configuration* cfg);

    /**
     * @brief Robots that should not be consider obstacles
     */
    std::vector<Robot*> excluded_robots;

    /**
     * @brief Locations to pretend are robot obstacles during evaluation
     */
    std::vector<geometry2d::Point> hypothetical_robot_locations;

    static ConfigDouble* kick_std_dev;

private:
    SystemState* system;

    /**
     * @return the width of the target segment in radians
     */
    static float get_target_angle(geometry2d::Point origin,
                                  geometry2d::Segment target);

    /**
     * @return Vector of valid robots on the field
     */
    std::vector<Robot*> get_valid_robots();

    /**
     * @brief Converts Robot position to polar in reference to the goal vector
     * @return <R, Theta>
     */
    static std::tuple<float, float> rect_to_polar(geometry2d::Point origin,
                                                  geometry2d::Point target,
                                                  geometry2d::Point obstacle);

    /**
     * @return List of all robots positions in polar coordinates
     */
    std::vector<std::tuple<float, float> > convert_robots_to_polar(
        geometry2d::Point origin, geometry2d::Point target);

    /**
     * @brief Initilizes ParallelGraident1DConfig based upon the robot locations
     * etc
     */
    static void init_gradient_configs(
        ParallelGradient1DConfig& pConfig,
        std::function<std::tuple<float, float>(float)>& func,
        const std::vector<float>& robotMeans,
        const std::vector<float>& robotStDevs, float boundaryLower,
        float boundaryUpper);

    static ConfigDouble* kick_mean;
    static ConfigDouble* robot_std_dev;
    static ConfigDouble* start_x_offset;
};
