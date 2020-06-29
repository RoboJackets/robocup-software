#include <gtest/gtest.h>

#include <fstream>

#include "Geometry2d/Pose.hpp"
#include "SystemState.hpp"
#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"
#include "planning/low_level/RRTUtil.hpp"
#include "planning/planner/CollectPlanner.hpp"
#include "planning/planner/MotionCommand.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/planner/SettlePlanner.hpp"
#include "planning/tests/TestingUtils.hpp"

/*
 * If these tests are failing, run again with the flag --gtest_break_on_failure
 * it makes debugging a lot easier
 */

using namespace Planning;
using namespace Geometry2d;
using namespace Planning::TestingUtils;

TEST(Planning, path_target_random) {
    std::mt19937 gen(1337);

    WorldState world_state;
    PathTargetPlanner planner;

    int failure_count = 0;
    for (int i = 0; i < 1000; i++) {
        ShapeSet obstacles;
        int numObstacles = random<int>(&gen, 2, 5);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(&gen, -2.0, 2.0), random(&gen, .5, 1.5)}, .2));
        }
        auto start = randomInstant(&gen);

        // If we start in an obstacle planning will trivially fail. We don't
        // care about this case.
        if (obstacles.hit(start.position())) {
            continue;
        }

        LinearMotionInstant goal = randomInstant(&gen).linear_motion();
        PlanRequest request{start,
                            PathTargetCommand{goal},
                            RobotConstraints{},
                            obstacles,
                            {},
                            {},
                            0,
                            &world_state,
                            2,
                            nullptr};
        Trajectory path = planner.plan(std::move(request));

        if (path.empty()) {
            failure_count++;
            continue;
        }

        if (!checkTrajectoryContinuous(path, RobotConstraints{})) {
            std::cout << "Saving to /tmp/out.csv" << std::endl;
            std::ofstream file("/tmp/out.csv");
            for (auto cursor = path.cursor_begin(); cursor.has_value();
                 cursor.advance(0.05s)) {
                auto instant = cursor.value();
                file << RJ::Seconds(instant.stamp - RJ::now()) << ", "
                     << instant.position().x() << ", " << instant.position().y()
                     << ", " << instant.linear_velocity().x() << ", "
                     << instant.linear_velocity().y() << std::endl;
            }
            EXPECT_FALSE(true);
        }

        // EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
    }

    EXPECT_LT(failure_count, 150)
        << "Failure rate should be less than 15 percent for path target";
}

TEST(Planning, collect_basic) {
    WorldState world_state;
    world_state.ball.position = Point{1, 1};
    world_state.ball.velocity = Point{0, 0};
    world_state.ball.timestamp = RJ::now();
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        CollectCommand{},
                        RobotConstraints{},
                        ShapeSet{},
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, collect_obstructed) {
    WorldState world_state;
    world_state.ball.position = Point{1, 1};
    world_state.ball.velocity = Point{0, 0};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{.5, .5}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        CollectCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, collect_pointless_obs) {
    WorldState world_state;
    world_state.ball.position = Point{1, 1};
    world_state.ball.velocity = Point{0, 0};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{2, 2}, .2));
    obstacles.add(std::make_shared<Circle>(Point{3, 3}, .2));
    obstacles.add(std::make_shared<Circle>(Point{-2, 3}, .2));
    obstacles.add(std::make_shared<Circle>(Point{0, 5}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        CollectCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, collect_moving_ball_quick) {
    WorldState world_state;
    world_state.ball.position = Point{-1, 1};
    world_state.ball.velocity = Point{-0.03, 0.3};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0, .5}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        CollectCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, collect_moving_ball_slow) {
    WorldState world_state;
    world_state.ball.position = Point{-1, 1};
    world_state.ball.velocity = Point{0, 0.1};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{-0.5, .5}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        CollectCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, collect_moving_ball_slow_2) {
    WorldState world_state;
    world_state.ball.position = Point{-1, 1};
    world_state.ball.velocity = Point{0.01, 0.05};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0, .5}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        CollectCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, collect_random) {
    std::mt19937 gen(1337);
    WorldState world_state;

    int failure_count = 0;

    for (int i = 0; i < 500; i++) {
        world_state.ball.position =
            Point{random(&gen, -1.5, 1.5), random(&gen, 2.0, 4.0)};
        world_state.ball.velocity =
            Point{random(&gen, -.3, .3), random(&gen, -1.0, 0.1)};
        world_state.ball.timestamp = RJ::now();
        ShapeSet obstacles;
        int numObstacles = random(&gen, 2, 5);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(&gen, -2.0, 2.0), random(&gen, 0.5, 1.5)}, .2));
        }
        PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                            CollectCommand{},
                            RobotConstraints{},
                            obstacles,
                            {},
                            {},
                            0,
                            &world_state,
                            2,
                            nullptr};
        CollectPlanner planner;
        Trajectory path = planner.plan(std::move(request));

        if (path.empty()) {
            failure_count++;
            continue;
        }

        EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
    }

    EXPECT_LT(failure_count, 50)
        << "Failure rate should be below 10 percent for collect";
}

TEST(Planning, settle_basic) {
    WorldState world_state;
    world_state.ball.position = Point{1, 1};
    world_state.ball.velocity = Point{-1, -1.5};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{.5, .5}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        SettleCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    SettlePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, settle_pointless_obs) {
    WorldState world_state;
    // Use some initial velocity, settle doesn't always work for non-moving
    // balls
    world_state.ball.position = Point{1, 3};
    world_state.ball.velocity = Point{-.1, -1};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{-1, 1.0}, .2));
    PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                        SettleCommand{},
                        RobotConstraints{},
                        obstacles,
                        {},
                        {},
                        0,
                        &world_state,
                        2,
                        nullptr};
    SettlePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    ASSERT_TRUE(!path.empty());
    EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
}

TEST(Planning, settle_random) {
    std::mt19937 gen(1337);
    WorldState world_state;

    int failure_count = 0;

    for (int i = 0; i < 500; i++) {
        world_state.ball.position =
            Point{random(&gen, -1.5, 1.5), random(&gen, 3.0, 4.0)};
        world_state.ball.velocity =
            Point{random(&gen, -.3, .3), random(&gen, -2.0, -1.0)};
        world_state.ball.timestamp = RJ::now();
        ShapeSet obstacles;
        int numObstacles = random(&gen, 0, 3);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(&gen, -2.0, 2.0), random(&gen, .5, 1.5)}, .2));
        }
        PlanRequest request{RobotInstant{{}, {}, RJ::now()},
                            SettleCommand{},
                            RobotConstraints{},
                            obstacles,
                            {},
                            {},
                            0,
                            &world_state,
                            2,
                            nullptr};
        SettlePlanner planner;
        Trajectory path = planner.plan(std::move(request));

        if (path.empty()) {
            failure_count++;
            continue;
        }

        EXPECT_TRUE(checkTrajectoryContinuous(path, RobotConstraints{}));
    }

    EXPECT_LT(failure_count, 100)
        << "Failure rate should be <20 percent for settle";
}

// todo: test Intercept, LineKick, WorldVel, EscapeObstacle