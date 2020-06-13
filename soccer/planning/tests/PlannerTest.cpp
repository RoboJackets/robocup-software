#include <gtest/gtest.h>

#include "Geometry2d/Pose.hpp"
#include "SystemState.hpp"
#include "planning/Trajectory.hpp"
#include "planning/Instant.hpp"
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
    WorldState world_state;
    PathTargetPlanner planner;
    for (int i = 0; i < 1000; i++) {
        ShapeSet obstacles;
        int numObstacles = (int)random(2, 5);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(-2, 2), random(.5, 1.5)}, .2));
        }
        RobotInstant goal = randomInstant();
        PlanRequest request{randomInstant(),
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
        assertPathContinuous(path, RobotConstraints{});
    }
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
    assertPathContinuous(path, RobotConstraints{});
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
    assertPathContinuous(path, RobotConstraints{});
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
    assertPathContinuous(path, RobotConstraints{});
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
    assertPathContinuous(path, RobotConstraints{});
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
    assertPathContinuous(path, RobotConstraints{});
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
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_random) {
    WorldState world_state;
    for (int i = 0; i < 50; i++) {
        world_state.ball.position = Point{random(-1.5, 1.5), random(2, 4)};
        world_state.ball.velocity = Point{random(-.3, .3), random(-1, .1)};
        world_state.ball.timestamp = RJ::now();
        ShapeSet obstacles;
        int numObstacles = (int)random(2, 5);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(-2, 2), random(.5, 1.5)}, .2));
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
        assertPathContinuous(path, RobotConstraints{});
    }
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
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, settle_pointless_obs) {
    WorldState world_state;
    world_state.ball.position = Point{1, 1};
    world_state.ball.velocity = Point{-.1, -.1};
    world_state.ball.timestamp = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0, .5}, .2));
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
assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, settle_random) {
    WorldState world_state;
    for (int i = 0; i < 50; i++) {
        world_state.ball.position = Point{random(-1.5, 1.5), random(2, 4)};
        world_state.ball.velocity = Point{random(-.3, .3), random(-1, -.2)};
        world_state.ball.timestamp = RJ::now();
        ShapeSet obstacles;
        int numObstacles = (int)random(2, 5);
        for (int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{random(-2, 2), random(.5, 1.5)}, .2));
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
        assertPathContinuous(path, RobotConstraints{});
    }
}

// todo: test Intercept, LineKick, WorldVel, EscapeObstacle