#include <gtest/gtest.h>
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "SystemState.hpp"
#include "Geometry2d/Pose.hpp"
#include "planning/tests/TestingUtils.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/SettlePlanner.hpp"
#include "planning/planner/CollectPlanner.hpp"
#include "planning/planner/MotionCommand.hpp"

/*
 * If these tests are failing, run again with the flag --gtest_break_on_failure
 * it makes debugging a lot easier
 */

using namespace Planning;
using namespace Geometry2d;
using namespace Planning::TestingUtils;

TEST(Planning, path_target_random) {
    Context context;
    PathTargetPlanner planner;
    for(int i = 0; i < 1000; i++) {
        context.state.logFrame = std::make_shared<Packet::LogFrame>();
        context.debug_drawer.setLogFrame(context.state.logFrame.get());

        ShapeSet obstacles;
        int numObstacles = (int) random(2, 5);
        for(int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(Point{random(-2, 2),
                                                         random(.5, 1.5)}, .2));
        }
        RobotInstant goal = randomInstant();
        PlanRequest request{&context, randomInstant(), PathTargetCommand{goal}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
        Trajectory path = planner.plan(std::move(request));
        assertPathContinuous(path, RobotConstraints{});
    }
}

TEST(Planning, collect_basic) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, ShapeSet{}, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_obstructed) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{.5,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_pointless_obs) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{2,2}, .2));
    obstacles.add(std::make_shared<Circle>(Point{3,3}, .2));
    obstacles.add(std::make_shared<Circle>(Point{-2, 3}, .2));
    obstacles.add(std::make_shared<Circle>(Point{0,5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_moving_ball_quick) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{-1,1};
    context.state.ball.vel = Point{-0.03,0.3};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_moving_ball_slow) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{-1,1};
    context.state.ball.vel = Point{0,0.1};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{-0.5,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_moving_ball_slow_2) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{-1,1};
    context.state.ball.vel = Point{0.01,0.05};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, collect_random) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    for(int i = 0; i < 50; i++) {
    context.state.ball.pos = Point{random(-1.5, 1.5), random(2, 4)};
    context.state.ball.vel = Point{random(-.3, .3), random(-1, .1)};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    int numObstacles = (int) random(2, 5);
    for(int j = 0; j < numObstacles; j++) {
    obstacles.add(std::make_shared<Circle>(Point{random(-2, 2),
                                                 random(.5, 1.5)}, .2));
    }
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CollectCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CollectPlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
    }
}

TEST(Planning, settle_basic) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{-1,-1.5};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{.5,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, SettleCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    SettlePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}


TEST(Planning, settle_pointless_obs) {
    Context context;
    context.state.logFrame = std::make_shared<Packet::LogFrame>();
    context.debug_drawer.setLogFrame(context.state.logFrame.get());
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{-.1,-.1};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, SettleCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    SettlePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, settle_random) {
    Context context;
    for(int i = 0; i < 50; i++) {
        context.state.logFrame = std::make_shared<Packet::LogFrame>();
        context.debug_drawer.setLogFrame(context.state.logFrame.get());
        context.state.ball.pos = Point{random(-1.5, 1.5), random(2, 4)};
        context.state.ball.vel = Point{random(-.3, .3), random(-1, -.2)};
        context.state.ball.time = RJ::now();
        ShapeSet obstacles;
        int numObstacles = (int) random(2, 5);
        for(int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(Point{random(-2, 2),
                                                         random(.5, 1.5)}, .2));
        }
        PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, SettleCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
        SettlePlanner planner;
        Trajectory path = planner.plan(std::move(request));
        assertPathContinuous(path, RobotConstraints{});
    }
}

//TODO(Ethan) test Intercept, LineKick, WorldVel, EscapeObstacle