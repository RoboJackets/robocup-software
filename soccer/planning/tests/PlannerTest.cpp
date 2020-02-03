#include <gtest/gtest.h>
#include "planning/trajectory/RRTUtil.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/CapturePlanner.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "SystemState.hpp"
#include "Geometry2d/Pose.hpp"
#include "planning/tests/TestingUtils.hpp"

/*
 * If these tests are failing, run again with the flag --gtest_break_on_failure
 * it makes debugging a lot easier
 */

using namespace Planning;
using namespace Geometry2d;
using namespace Planning::TestingUtils;

TEST(Planning, path_target_random) {
    Context context;
    for(int i = 0; i < 1000; i++) {
        ShapeSet obstacles;
        int numObstacles = (int) random(2, 5);
        for(int j = 0; j < numObstacles; j++) {
            obstacles.add(std::make_shared<Circle>(Point{random(-2, 2),
                                                         random(.5, 1.5)}, .2));
        }
        RobotInstant goal = randomInstant();
        PlanRequest request{&context, randomInstant(), PathTargetCommand{goal}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
        PathTargetPlanner planner;
        Trajectory path = planner.plan(std::move(request));
        assertPathContinuous(path, RobotConstraints{});
    }
}

TEST(Planning, capture_basic) {
    Context context;
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, ShapeSet{}, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
//    printf("produced path with %d instants of duration %.3f sec\n", path.num_instants(), path.duration().count());
//    printf("iterating with TrajectoryIterator...\n");
//    for(auto it = path.iterator(path.begin_time(), 100ms); it.hasValue(); ++it) {
//        printf("(%.2f, %.2f)\n", (*it).pose.position().x(), (*it).pose.position().y());
//    }
//    printf("iterating with normal iterator...\n");
//    for(auto it = path.instants_begin(); it != path.instants_end(); ++it) {
//        printf("(%.3f, %.3f, %.3f) (%.3f, %.3f, %.3f) %.6f\n",
//               (*it).pose.position().x(), (*it).pose.position().y(), (*it).pose.heading(),
//               (*it).velocity.linear().x(), (*it).velocity.linear().y(), (*it).velocity.angular(),
//               RJ::Seconds((*it).stamp-path.begin_time()).count()
//        );
//    }
}


TEST(Planning, capture_obstructed) {
    Context context;
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{.5,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, capture_pointless_obs) {
    Context context;
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{0,0};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{2,2}, .2));
    obstacles.add(std::make_shared<Circle>(Point{3,3}, .2));
    obstacles.add(std::make_shared<Circle>(Point{-2, 3}, .2));
    obstacles.add(std::make_shared<Circle>(Point{0,5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, capture_moving_ball_away_quick) {
    Context context;
    context.state.ball.pos = Point{-1,1};
    context.state.ball.vel = Point{-0.03,0.3};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}
TEST(Planning, capture_moving_ball_away_slow) {
    Context context;
    context.state.ball.pos = Point{-1,1};
    context.state.ball.vel = Point{0.01,0.05};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, capture_moving_ball_incoming) {
    Context context;
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{-1,-1.5};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{.5,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, capture_moving_ball_away_with_obstacles) {
    Context context;
    context.state.ball.pos = Point{-1,1};
    context.state.ball.vel = Point{0,0.1};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{-0.5,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, capture_moving_ball_incoming_with_obstacles) {
    Context context;
    context.state.ball.pos = Point{1,1};
    context.state.ball.vel = Point{-.1,-.1};
    context.state.ball.time = RJ::now();
    ShapeSet obstacles;
    obstacles.add(std::make_shared<Circle>(Point{0,.5}, .2));
    PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
    CapturePlanner planner;
    Trajectory path = planner.plan(std::move(request));
    assertPathContinuous(path, RobotConstraints{});
}

TEST(Planning, capture_random) {
    Context context;
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
        PlanRequest request{&context, RobotInstant{{},{}, RJ::now()}, CaptureCommand{}, RobotConstraints{}, Trajectory{{}}, obstacles, {}, 2};
        CapturePlanner planner;
        Trajectory path = planner.plan(std::move(request));
        assertPathContinuous(path, RobotConstraints{});
    }
}