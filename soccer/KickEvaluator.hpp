#pragma once

#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Point.hpp>
#include "Robot.hpp"
#include "SystemState.hpp"

#include <math.h>
#include <tuple>

/**
 * @brief KickEvaluator calculates the chance a kick succeeds
 */
class KickEvaluator {
public:
    /**
     * @brief Constructor
     * @param systemState pointer to global system state object
     */
    KickEvaluator(SystemState* systemState);

    /**
     * @brief Evalutes kick to target point
     * @param origin The starting point of the kick
     * @param target The end point of the kick
     * @param targetWidth Width to apply to the target
     * @return Percent chance to succeed
     */
    float eval_pt_to_pt(Geometry2d::Point origin,
                        Geometry2d::Point target,
                        float targetWidth);

    /**
     * @brief Evaluates kick to robot sized segment
     * @param origin The starting point of the kick
     * @param target The end point of the kick
     * @return Percent chance to succeed
     */
    float eval_pt_to_robot(Geometry2d::Point origin,
                           Geometry2d::Point target);

    /**
     * @brief Evaluates kick to the center of the oponents goal
     * @param origin The starting point of the shot
     * @return Percent chance to succeed
     */
    float eval_pt_to_opp_goal(Geometry2d::Point origin);

    /**
     * @brief Evaluates kick to the center of our team's goal
     * @param origin The starting point of the shot
     * @return Percent chance to succeed
     */
    float eval_pt_to_our_goal(Geometry2d::Point origin);

    /**
     * @brief Evalutes kick to target segment
     * @param origin The starting point of the kick
     * @param target The target segment to aim at
     * @return Percent chance to succeed
     */
    float eval_pt_to_seg(Geometry2d::Point origin,
                         Geometry2d::Segment target);

    /**
     * @brief Initializes configurable fields
     * @note SEe configuration documentation for details
     */
    static void createConfiguration(Configuration* cfg);

    /**
     * @brief Robots that should not be consider obstacles
     */
    std::vector<Robot*> excluded_robots;

    /**
     * @brief Locations to pretend are robot obstacles during evaluation
     */
    std::vector<Geometry2d::Point> hypothetical_robot_locations;

    /**
     * @brief Number of rays to extend from the origin
     * @note As number of rays -> inf, real error -> 0
     */
    float number_of_rays = 16;

    /**
     * @brief Removes any robots not within this angle of target
     */
    float max_delta_angle = 0.7f * M_PI_2;

private:
    SystemState* system;

    std::tuple<float, float> rect_to_polar(Geometry2d::Point origin,
                                           Geometry2d::Point target,
                                           Geometry2d::Point obstacle);

    // Max error of 0.05 at x = -1.5
    float fast_exp(float x);

    static ConfigDouble* robot_angle_filter_limit;
    static ConfigDouble* kick_std_dev;
    static ConfigDouble* num_rays;

    static ConfigDouble* kernal_width_coefficient;
    static ConfigDouble* distance_coefficient;
};