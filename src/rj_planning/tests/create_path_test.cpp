#include "rj_planning/primitives/create_path.hpp"

#include <fstream>
#include <iostream>
#include <random>

#include <gtest/gtest.h>

#include "rj_planning/trajectory_utils.hpp"
#include "rj_planning/utils/testing_utils.hpp"

using namespace rj_geometry;

namespace planning {

TEST(CreatePath, smoke_test_efficiency) {
    MotionConstraints mot;

    ShapeSet obs;

    {
        RJ::Time t0 = RJ::now();
        CreatePath::rrt(LinearMotionInstant{Point(0, 0)}, LinearMotionInstant{Point(1, 1)}, mot,
                        RJ::now(), obs);
        std::cout << "time for CreatePath::rrt direct: %.6f\n"
                  << RJ::Seconds(RJ::now() - t0).count() << std::endl;
    }

    {
        RJ::Time t0 = RJ::now();
        obs.add(std::make_shared<Circle>(Point{.5, .5}, 0.2));
        CreatePath::rrt(LinearMotionInstant{Point()}, LinearMotionInstant{Point(1, 1)}, mot,
                        RJ::now(), obs);
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
                Point{TestingUtils::random(&gen, -2.0, 2.0), TestingUtils::random(&gen, 2.0, 3.0)},
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
            path = CreatePath::rrt(start, goal, constraints.mot, RJ::now(), obstacles);
            if (path.empty()) {
                fails++;
            }
            ASSERT_TRUE(j != kNumTries - 1);
        }
        EXPECT_TRUE(TestingUtils::check_trajectory_continuous(path, constraints));
    }

    double success_rate = (double)(kIterations) / (kIterations + fails);
    std::cout << "CreatePath::rrt() Success Rate: %.6f\n" << success_rate;
    EXPECT_GT(success_rate, 0.75);
}

TEST(CreatePath, intermediate_creation_time) {
    std::mt19937 gen(1337);

    constexpr int kIterations = 10000;
    RobotConstraints constraints;
    const FieldDimensions* field_dimensions = &FieldDimensions::current_dimensions;

    double average_time;
    std::cout << "Saving to intermediate_creation.out and intermediate_traversal.out\n";
    std::ofstream file("intermediate_creation.out");
    std::ofstream tfile("intermediate_traversal.out");
    std::ofstream rfile("rrt_creation.out");
    std::ofstream rtfile("rrt_traversal.out");

    for (int i = 0; i < kIterations; i++) {
        Point start_point{TestingUtils::random(&gen, -3.0, 3.0),
                          TestingUtils::random(&gen, 5.0, 5.5)};
        Point start_velocity{};
        LinearMotionInstant start{start_point, start_velocity};

        Point end_point{TestingUtils::random(&gen, -3.0, 3.0),
                        TestingUtils::random(&gen, 0.5, 1.0)};
        Point end_velocity{};
        LinearMotionInstant goal{end_point, end_velocity};

        ShapeSet obstacles;
        int num_obstacles = 5;
        double obst_size = 0.2;
        for (int j = 0; j < num_obstacles; j++) {
            auto direction = end_point - start_point;
            auto length = direction.mag();
            auto t = TestingUtils::random(&gen, 0.4 / length, 1 - 0.4 / length);

            auto base_point = start_point + t * direction.normalized();
            auto perp_direction = direction.rotate(degrees_to_radians(90.));
            auto random_distance = TestingUtils::random(&gen, 0., obst_size - 0.01);
            auto offset = perp_direction.normalized() * random_distance;
            auto random_sign = TestingUtils::random(&gen, 0., 1.) > 0.5 ? 1 : -1;
            offset *= random_sign;
            auto obst_center = base_point + offset;

            obstacles.add(std::make_shared<Circle>(obst_center, obst_size));
        }

        auto start_time = RJ::now();
        Trajectory traj = CreatePath::intermediate(start, goal, constraints.mot, RJ::now(),
                                                   obstacles, {}, field_dimensions, 0);
        double nanos = (RJ::now() - start_time).count();
        file << "CreatePath::intermediate() Time: " << nanos / 1e6 << " ms\n";
        tfile << "CreatePath::intermediate() Time: "
              << (traj.end_time() - traj.begin_time()).count() / 1e9 << " s\n";

        start_time = RJ::now();
        traj = CreatePath::rrt(start, goal, constraints.mot, RJ::now(), obstacles);
        nanos = (RJ::now() - start_time).count();
        rfile << "CreatePath::rrt() Time: " << nanos / 1e6 << " ms\n";
        rtfile << "CreatePath::rrt() Time: " << (traj.end_time() - traj.begin_time()).count() / 1e9
               << " s\n";

        average_time += nanos;
    }
    file.close();

    average_time /= kIterations;
    std::cout << "CreatePath::intermediate() Average Time: " << average_time << " ns\n";
    EXPECT_GT(average_time, 0);
}

}  // namespace planning
