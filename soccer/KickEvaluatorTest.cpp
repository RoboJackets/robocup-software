#include <gtest/gtest.h>
#include "KickEvaluator.hpp"
#include "SystemState.hpp"

#include <stdlib.h>

using namespace Geometry2d;

TEST(KickEvaluator, eval_pt_to_our_goal) {
    SystemState state;
    OurRobot* obstacleBot = state.self[0];
    obstacleBot->visible = true;
    obstacleBot->pos = Point(1, 1);

    KickEvaluator kickEval(&state);
    std::pair<Point, double> pt_to_our_goal = kickEval.eval_pt_to_our_goal(Point(0, 2));

    KickEvaluatorArgs test(0, 0.1, {0.1}, {0.1}, {1}, -2, 2);
    std::tuple<double, double> res = KickEvaluator::eval_calculation(0, &test);
    double c = std::get<0>(res);
    double d = std::get<1>(res);
    std::cout << c << " Value, Derivative " << d << std::endl;
}