#pragma once

#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Point.hpp>
#include "Robot.hpp"
#include "SystemState.hpp"

#include "optimization/ParallelGradientAscent1D.hpp"
#include "optimization/ParallelGradient1DConfig.hpp"
#include "optimization/FunctionArgs.hpp"
#include "optimization/KickEvaluatorArgs.hpp"

#include <vector>
#include <memory>

// Point along target segment to aim at
// % Chance of success
using KickResults = std::pair<Geometry2d::Point, double>;

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
    KickResults eval_pt_to_pt(Geometry2d::Point origin, Geometry2d::Point target,
                        float targetWidth);

    /**
     * @brief Evaluates kick to target robot
     * @param origin, Starting point of the kick
     * @param target, Target robot location
     * @return Results of calculations
     */
    KickResults eval_pt_to_robot(Geometry2d::Point origin, Geometry2d::Point target);

    /**
     * @brief Evaluates kick to opponent goal
     * @param origin, Starting point of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_opp_goal(Geometry2d::Point origin);

    /**
     * @brief Evaluates kick to our goal
     * @return Results of calculations
     */
    KickResults eval_pt_to_our_goal(Geometry2d::Point origin);

    /**
     * @brief Evaluates kick to target segment
     * @param origin, Starting point of the kick
     * @param target, End segment of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_seg(Geometry2d::Point origin, Geometry2d::Segment target);

    /**
     * @brief Evaluates closed form solution of the KickEvaluation problem
     * @return F(X), F'(X)
     */
    static std::tuple<double, double> eval_calculation(double x, FunctionArgs* fArgs);

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
    std::vector<Geometry2d::Point> hypothetical_robot_locations;
private:
    SystemState* system;

    double get_reference_angle(Geometry2d::Point origin, Geometry2d::Segment target);

    std::vector<Robot*> get_valid_robots();

    std::tuple<double, double> rect_to_polar(Geometry2d::Point origin,
                                             Geometry2d::Point target,
                                             Geometry2d::Point obstacle);

    std::vector< std::tuple<double, double> > convert_robots_to_polar(Geometry2d::Point origin,
                                                                      Geometry2d::Point target);

    ParallelGradient1DConfig init_gradient_configs(KickEvaluatorArgs* keArgs);

    static ConfigDouble* kick_std_dev;
    static ConfigDouble* kick_mean;
    static ConfigDouble* robot_std_dev;
    static ConfigDouble* robot_dist_scale;
    static ConfigDouble* start_x_offset;
};