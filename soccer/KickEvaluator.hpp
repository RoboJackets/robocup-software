#pragma once

#include <Geometry2d/Point.hpp>
#include "Robot.hpp"
#include "SystemState.hpp"

/**
 * @brief KickEvaluator calculates the chance a pass succeeds
 */
class KickEvaluator {
public:
    /**
     * @brief Constructor
     * @param systemState pointer to global system state object
     */
    KickEvaluator(SystemState* systemState);

    /**
     * @brief Evalutes pass to target point
     * @param origin The starting point of the pass
     * @param target The end point of the pass
     * @param targetWidth Width to apply to the target
     * @return Percent chance to succeed
     */
    double eval_pt_to_pt(Geometry2d::Point origin,
                        Geometry2d::Point target,
                        float targetWidth);

    /**
     * @brief Robots that should be consider obstacles
     */
    std::vector<Robot*> excluded_robots;

private:
    SystemState* system;
};