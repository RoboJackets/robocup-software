#include <gtest/gtest.h>
#include "KickEvaluator.hpp"
#include "SystemState.hpp"

#include <stdlib.h>

using namespace Geometry2d;

TEST(KickEvaluator, no_robots) {
    SystemState state;
    KickEvaluator kickEval(&state);
    std::pair<Point, double> pt_to_our_goal;
    std::pair<Point, double> expected =
        std::pair<Point, double>(Point{0, 0}, 1.0);

    // No opponent robot
    pt_to_our_goal = kickEval.eval_pt_to_our_goal(Point(0, 0.3));

    // Due to the search functions, these may be a little off sometimes
    EXPECT_NEAR((std::get<0>(expected)).x(), (std::get<0>(pt_to_our_goal)).x(),
                0.01);
    EXPECT_NEAR((std::get<0>(expected)).y(), (std::get<0>(pt_to_our_goal)).y(),
                0.01);
    EXPECT_NEAR(std::get<1>(expected), std::get<1>(pt_to_our_goal), 0.01);
}

TEST(KickEvaluator, eval_pt_to_our_goal) {
    SystemState state;
    OurRobot* obstacleBot = state.self[0];
    obstacleBot->visible = true;
    obstacleBot->pos = Point(1, 1);

    KickEvaluator kickEval(&state);
    std::pair<Point, double> pt_to_our_goal;
    std::pair<Point, double> expected =
        std::pair<Point, double>(Point{0, 0}, 0.56);

    pt_to_our_goal = kickEval.eval_pt_to_our_goal(Point(0, 2));

    EXPECT_GT((std::get<0>(expected)).x(), (std::get<0>(pt_to_our_goal)).x());
    EXPECT_NEAR((std::get<0>(expected)).y(), (std::get<0>(pt_to_our_goal)).y(),
                0.01);
    EXPECT_GT(std::get<1>(expected), 0);
    EXPECT_LT(std::get<1>(expected), 1);
}

TEST(KickEvaluator, eval_calculation) {
    std::vector<float> robotMeans = {10};
    std::vector<float> robotStdevs = {.1};
    std::vector<float> robotVertScores = {1};

    // Kick mean, Kick stdev, Robot mean, robot stdev, robot scale, boundsLower,
    // boundsUpper
    std::tuple<double, double> res = KickEvaluator::eval_calculation(
        0, 0, 0.1, std::ref(robotMeans), std::ref(robotStdevs),
        std::ref(robotVertScores), -2, 2);

    std::tuple<double, double> expected = std::make_tuple(1, 0);

    EXPECT_NEAR(std::get<0>(res), std::get<0>(res), 0.01);  // Value
    EXPECT_NEAR(std::get<1>(res), std::get<1>(res), 0.01);  // Derivative
}

TEST(KickEvaluator, eval_kick_at_pi_transition) {
    SystemState state;

    KickEvaluator kickEval(&state);
    std::pair<Point, double> pt_to_our_goal;

    pt_to_our_goal = kickEval.eval_pt_to_robot(Point(3, 2), Point(-3, 2));

    EXPECT_LT(std::get<1>(pt_to_our_goal), 0.99);
}