#pragma once
#include "Geometry2d/Point.hpp"
#include "planning/planner/PlanRequest.hpp"

namespace Planning {

// Try find best point to intercept using brute force method
// where we check ever X distance along the ball velocity vector
//
// Disallow points outside the field
Geometry2d::Point bruteForceGoal(const PlanRequest& request, Geometry2d::Point avgBallVel,
        double searchStart, double searchEnd, double searchInc);


}