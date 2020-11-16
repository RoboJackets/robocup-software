#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/segment.hpp>
#include <rj_param_utils/param.hpp>
#include <functional>
#include <vector>

#include "robot.hpp"
#include "system_state.hpp"
#include "optimization/parallel_gradient_1d_config.hpp"
#include "optimization/parallel_gradient_ascent_1d.hpp"

constexpr auto kickEvaluatorParamModule = "kick_evaluator";

DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, kick_std_dev)
DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, kick_mean)
DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, robot_std_dev)
DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, start_x_offset)

// < [Point along target segment to aim at], [% Chance of success] >
using KickResults = std::pair<rj_geometry::Point, float>;

/**
 * @brief Finds the best position to kick to and the chance of success
 */
class KickEvaluator {
public:
    /**
     * @brief Constructor
     * @param system_state, pointer to the global system state object
     */
    KickEvaluator(SystemState* system_state);

    /**
     * @brief Evaluates kick to target point
     * @param origin, Starting point of the kick
     * @param target, End point of the kick
     * @param target_width, Width of target in radians
     * @return Results of calculations
     */
    KickResults eval_pt_to_pt(rj_geometry::Point origin,
                              rj_geometry::Point target, float target_width);

    /**
     * @brief Evaluates kick to target robot
     * @param origin, Starting point of the kick
     * @param target, Target robot location
     * @return Results of calculations
     */
    KickResults eval_pt_to_robot(rj_geometry::Point origin,
                                 rj_geometry::Point target);

    /**
     * @brief Evaluates kick to opponent goal
     * @param origin, Starting point of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_opp_goal(rj_geometry::Point origin);

    /**
     * @brief Evaluates kick to our goal
     * @param origin, Starting point of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_our_goal(rj_geometry::Point origin);

    /**
     * @brief Evaluates kick to target segment
     * @param origin, Starting point of the kick
     * @param target, End segment of the kick
     * @return Results of calculations
     */
    KickResults eval_pt_to_seg(rj_geometry::Point origin,
                               rj_geometry::Segment target);

    /**
     * @brief Evaluates closed form solution of the KickEvaluation problem
     * @param x, Location to run at
     * @param kmean, Kick mean
     * @param kstdev, Kick standard deviation
     * @param robot_means, vector of robot angle locations
     * @param robot_st_devs, vector of robot movement standard deviations
     * @param robot_vert_scales, vector of how much to scale the height
     * @param b_left, left boundary angle
     * @param b_right, right boundary angle
     * @return F(X), F'(X)
     */
    static std::tuple<float, float> eval_calculation(
        float x, float kmean, float kstdev,
        const std::vector<float>& robot_means,
        const std::vector<float>& robot_st_devs,
        const std::vector<float>& robot_vert_scales, float b_left, float b_right);

    /**
     * @brief Initializes configurable fields
     * @note See configuration documentation for details
     */
    static void create_configuration(Configuration* cfg);

    /**
     * @brief Robots that should not be consider obstacles
     */
    std::vector<Robot*> excluded_robots;

    /**
     * @brief Locations to pretend are robot obstacles during evaluation
     */
    std::vector<rj_geometry::Point> hypothetical_robot_locations;

    // DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, kick_std_dev)

private:
    SystemState* system_;

    /**
     * @return the width of the target segment in radians
     */
    static float get_target_angle(rj_geometry::Point origin,
                                  rj_geometry::Segment target);

    /**
     * @return Vector of valid robots on the field
     */
    std::vector<Robot*> get_valid_robots();

    /**
     * @brief Converts Robot position to polar in reference to the goal vector
     * @return <R, Theta>
     */
    static std::tuple<float, float> rect_to_polar(rj_geometry::Point origin,
                                                  rj_geometry::Point target,
                                                  rj_geometry::Point obstacle);

    /**
     * @return List of all robots positions in polar coordinates
     */
    std::vector<std::tuple<float, float> > convert_robots_to_polar(
        rj_geometry::Point origin, rj_geometry::Point target);

    /**
     * @brief Initilizes ParallelGraident1DConfig based upon the robot locations
     * etc
     */
    static void init_gradient_configs(
        ParallelGradient1DConfig& p_config,
        std::function<std::tuple<float, float>(float)>& func,
        const std::vector<float>& robot_means,
        const std::vector<float>& robot_st_devs, float boundary_lower,
        float boundary_upper);

    
 //    DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, kick_mean)
	// DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, robot_std_dev)
	// DECLARE_NS_FLOAT64(kickEvaluatorParamModule, kick_evaluator, start_x_offset)
};