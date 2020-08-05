#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "planning/primitives/CreatePath.hpp"
#include "planning/tests/TestingUtils.hpp"

using namespace Geometry2d;

namespace Planning {

TEST(CreatePath, smoke_test_efficiency) {
    MotionConstraints mot;

    ShapeSet obs;

    {
        RJ::Time t0 = RJ::now();
        CreatePath::rrt(LinearMotionInstant{Point(0, 0)},
                        LinearMotionInstant{Point(1, 1)}, mot, RJ::now(), obs);
        std::cout << "time for CreatePath::rrt direct: %.6f\n"
                  << RJ::Seconds(RJ::now() - t0).count() << std::endl;
    }

    {
        RJ::Time t0 = RJ::now();
        obs.add(std::make_shared<Circle>(Point{.5, .5}, 0.2));
        CreatePath::rrt(LinearMotionInstant{Point()},
                        LinearMotionInstant{Point(1, 1)}, mot, RJ::now(), obs);
        std::cout << "time for CreatePath::rrt obstructed: %.6f\n"
                  << RJ::Seconds(RJ::now() - t0).count() << std::endl;
    }
}

TEST(CreatePath, infinitesimal_rrt) {
    RJ::Time time = RJ::now();
    RobotInstant start{Pose{{}, .1}, Twist{}, time};
    Trajectory a = CreatePath::rrt(start.linear_motion(), start.linear_motion(),
                                   MotionConstraints{}, start.stamp, {});
    ASSERT_FALSE(a.empty());
    ASSERT_TRUE(a.num_instants() == 1);
    ASSERT_NEAR(a.duration().count(), 0.0, 1e-6);
}

TEST(CreatePath, success_rate) {
    std::mt19937 gen(1337);

    int fails = 0;
    constexpr int kIterations = 1000;
    constexpr int kNumTries = 300;
    RobotConstraints constraints;

    for (int i = 0; i < kIterations; i++) {
        ShapeSet obstacles;
        int num_obstacles = TestingUtils::random(&gen, 2, 5);
        for (int j = 0; j < num_obstacles; j++) {
            obstacles.add(std::make_shared<Circle>(
                Point{TestingUtils::random(&gen, -2.0, 2.0),
                      TestingUtils::random(&gen, 2.0, 3.0)},
                .2));
        }

        Point start_point{TestingUtils::random(&gen, -3.0, 3.0),
                          TestingUtils::random(&gen, 5.0, 5.5)};
        Point start_velocity{TestingUtils::random(&gen, -.5, .5),
                             TestingUtils::random(&gen, -.5, .5)};
        LinearMotionInstant start{start_point, start_velocity};

        Point end_point{TestingUtils::random(&gen, -3.0, 3.0),
                        TestingUtils::random(&gen, 0.5, 1.0)};
        Point end_velocity{TestingUtils::random(&gen, -.5, .5),
                           TestingUtils::random(&gen, -.5, .5)};
        LinearMotionInstant goal{end_point, end_velocity};

        Trajectory path{{}};
        for (int j = 0; j < kNumTries && path.empty(); j++) {
            path = CreatePath::rrt(start, goal, constraints.mot, RJ::now(),
                                   obstacles);
            if (path.empty()) {
                fails++;
            }
            ASSERT_TRUE(j != kNumTries - 1);
        }
        EXPECT_TRUE(TestingUtils::checkTrajectoryContinuous(path, constraints));
    }

    double success_rate = (double)(kIterations) / (kIterations + fails);
    std::cout << "CreatePath::rrt() Success Rate: %.6f\n" << success_rate;
    EXPECT_GT(success_rate, 0.75);
}

}  // namespace Planning
