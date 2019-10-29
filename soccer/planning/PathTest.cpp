#include <iostream>
#include <gtest/gtest.h>
#include <planning/InterpolatedPath.hpp>
#include <planning/CompositePath.hpp>

using namespace std;
using namespace Geometry2d;

namespace Planning {

TEST(Path, nearestSegment) {
    Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

    InterpolatedPath path;
    path.waypoints.emplace_back(Pose(p0, 0), Twist::Zero(), 0s);
    path.waypoints.emplace_back(Pose(p1, 0), Twist::Zero(), 0s);
    path.waypoints.emplace_back(Pose(p2, 0), Twist::Zero(), 0s);
    path.waypoints.emplace_back(Pose(p3, 0), Twist::Zero(), 0s);

    Segment actSeg = path.nearestSegment(Point(0.5, -0.5));
    EXPECT_FLOAT_EQ(p0.x(), actSeg.pt[0].x());
    EXPECT_FLOAT_EQ(p0.y(), actSeg.pt[0].y());
    EXPECT_FLOAT_EQ(p1.x(), actSeg.pt[1].x());
    EXPECT_FLOAT_EQ(p1.y(), actSeg.pt[1].y());
}

TEST(InterpolatedPath, evaluate) {
    Point p0(1, 1), p1(1, 2), p2(2, 2);

    InterpolatedPath path;
    path.waypoints.emplace_back(Pose(p0, 0), Twist::Zero(), 0s);
    path.waypoints.emplace_back(Pose(p1, 0), Twist(p1, 0), 3s);
    path.waypoints.emplace_back(Pose(p2, 0), Twist::Zero(), 6s);

    // Pose and velocity should take on correct values at and near waypoints
    {
        auto pt = path.evaluate(0s);
        ASSERT_TRUE(pt);
        EXPECT_FLOAT_EQ(pt->motion.pos.x(), p0.x());
        EXPECT_FLOAT_EQ(pt->motion.pos.y(), p0.y());
        EXPECT_FLOAT_EQ(pt->motion.vel.x(), 0);
        EXPECT_FLOAT_EQ(pt->motion.vel.y(), 0);
    }

    {
        auto pt = path.evaluate(1e-6s);
        ASSERT_TRUE(pt);
        EXPECT_NEAR(pt->motion.pos.x(), p0.x(), 1e-3);
        EXPECT_NEAR(pt->motion.pos.y(), p0.y(), 1e-3);
        EXPECT_NEAR(pt->motion.vel.x(), 0, 1e-3);
        EXPECT_NEAR(pt->motion.vel.y(), 0, 1e-3);
    }

    {
        auto pt = path.evaluate(3s);
        ASSERT_TRUE(pt);
        EXPECT_FLOAT_EQ(pt->motion.pos.x(), p1.x());
        EXPECT_FLOAT_EQ(pt->motion.pos.y(), p1.y());
        EXPECT_FLOAT_EQ(pt->motion.vel.x(), p1.x());
        EXPECT_FLOAT_EQ(pt->motion.vel.y(), p2.y());
    }

    {
        auto pt = path.evaluate(3s + 1e-6s);
        ASSERT_TRUE(pt);
        EXPECT_NEAR(pt->motion.pos.x(), p1.x(), 1e-3);
        EXPECT_NEAR(pt->motion.pos.y(), p1.y(), 1e-3);
        EXPECT_NEAR(pt->motion.vel.x(), p1.x(), 1e-3);
        EXPECT_NEAR(pt->motion.vel.y(), p2.y(), 1e-3);
    }

    // Check that velocity matches expected
    for (RJ::Seconds t = 0s; t + 1e-6s < 6s; t += 5ms) {
        auto dt = 1e-6s;
        auto pt = path.evaluate(t);
        auto pt2 = path.evaluate(t + dt);
        ASSERT_TRUE(pt);
        ASSERT_TRUE(pt2);
        auto velocity = (pt->twist() + pt2->twist()) / 2;
        Twist finite_difference(Eigen::Vector3d(pt2->pose() - pt->pose()) /
                                RJ::numSeconds(dt));
        ASSERT_LT(Eigen::Vector3d(velocity - finite_difference)
                      .lpNorm<Eigen::Infinity>(),
                  1e-3);
    }

    // path should be invalid and at end state when t > duration
    auto out = path.evaluate(1000s);
    ASSERT_FALSE(out);
}

TEST(InterpolatedPath, subPath1) {
    InterpolatedPath path;
    path.waypoints.emplace_back(Pose(1, 1, 0), Twist(0, 0, 0), 0s);
    path.waypoints.emplace_back(Pose(1, 2, 0), Twist(1, 1, 0), 3s);
    path.waypoints.emplace_back(Pose(1, 3, 0), Twist(0, 0, 0), 6s);

    //  test invalid parameters to subPath()
    EXPECT_THROW(path.subPath(-1s, 5s), invalid_argument);
    EXPECT_THROW(path.subPath(8s, 20s),
                 invalid_argument);  //  start time beyond bounds of path
    EXPECT_THROW(path.subPath(5s, -2s), invalid_argument);

    //  make a subpath that cuts off one second from the start and end of the
    //  original
    unique_ptr<Path> subPath = path.subPath(1s, 5s);
    RJ::Seconds midTime((5 - 1) / 2.0);
    auto mid = subPath->evaluate(midTime);
    ASSERT_TRUE(mid);
    EXPECT_FLOAT_EQ(Point(1, 1).x(), mid->motion.vel.x());

    //  mid velocity of subpath should be the same as velocity of original path
    EXPECT_FLOAT_EQ(Point(1, 1).y(), mid->motion.vel.y());

    EXPECT_FLOAT_EQ(1, mid->motion.pos.x());
    EXPECT_FLOAT_EQ(2, mid->motion.pos.y());

    //  test the subpath at t = 0
    auto start = subPath->evaluate(0s);
    ASSERT_TRUE(start);

    //  the starting velocity of the subpath should be somewhere between the 0
    //  and the velocity at the middle
    EXPECT_GT(start->motion.vel.mag(), 0);
    EXPECT_LT(start->motion.vel.mag(), Point(1, 1).mag());

    //  the starting position of the subpath should be somewhere between the
    //  start pos of the original path and the middle point
    EXPECT_GT(start->motion.pos.y(), 1);
    EXPECT_LT(start->motion.pos.x(), 2);
}

TEST(InterpolatedPath, subpath2) {
    // Create a test path
    InterpolatedPath path;
    path.waypoints.emplace_back(Pose(1, 0, 0), Twist(0, 0, 0), 0s);
    path.waypoints.emplace_back(Pose(1, 2, 0), Twist(-1, -1, 0), 1s);
    path.waypoints.emplace_back(Pose(-2, 19, 0), Twist(1, 1, 0), 3s);
    path.waypoints.emplace_back(Pose(1, 6, 0), Twist(0, 0, 0), 9s);

    // Create 6 subPaths of length 1.5
    vector<unique_ptr<Path>> subPaths;
    RJ::Seconds diff = 1500ms;
    for (RJ::Seconds i = 0ms; i < 9s; i += diff) {
        subPaths.push_back(path.subPath(i, i + diff));
    }

    auto x = path.evaluate(4.5s);

    // Compare the subPaths to the original path and check that the results of
    // evaluating the paths are close enough
    for (int i = 0; i < 6; i++) {
        for (auto j = 0ms; j < 1500ms; j += 1ms) {
            auto time = i * 1500ms + j;
            auto org = path.evaluate(time);
            auto sub = subPaths[i]->evaluate(j);

            ASSERT_TRUE(org);
            ASSERT_TRUE(sub);
            EXPECT_NEAR(org->motion.vel.x(), sub->motion.vel.x(), 0.000001)
                << "i+j=" << to_string(time);
            EXPECT_NEAR(org->motion.vel.y(), sub->motion.vel.y(), 0.000001)
                << "i+j=" << to_string(time);
            EXPECT_NEAR(org->motion.pos.x(), sub->motion.pos.x(), 0.000001)
                << "i+j=" << to_string(time);
            EXPECT_NEAR(org->motion.pos.y(), sub->motion.pos.y(), 0.00001)
                << "i+j=" << to_string(time);
        }
    }
}

TEST(CompositePath, CompositeSubPath) {
    // Create a test path
    InterpolatedPath path;
    path.waypoints.emplace_back(Pose(1, 0, 0), Twist(0, 0, 0), 0s);
    path.waypoints.emplace_back(Pose(1, 2, 0), Twist(-1, -1, 0), 1s);
    path.waypoints.emplace_back(Pose(-2, 19, 0), Twist(1, 1, 0), 3s);
    path.waypoints.emplace_back(Pose(1, 6, 0), Twist(0, 0, 0), 9s);

    // Create 6 subPaths and rejoin them together into one compositePath
    CompositePath compositePath;
    auto diff = 1500ms;
    for (auto i = 0ms; i < 7500ms; i += diff) {
        compositePath.append(path.subPath(i, i + diff));
    }
    compositePath.append(path.subPath(7500ms));

    // Compare that the compositePath and origonal path are mostly equal
    for (RJ::Seconds i = 0ms; i <= 10s; i += 1ms) {
        auto org = path.evaluate(i);
        auto sub = compositePath.evaluate(i);
        if (!org && !sub) break;

        ASSERT_TRUE(org);
        ASSERT_TRUE(sub);
        EXPECT_NEAR(org->motion.vel.x(), sub->motion.vel.x(), 0.000001)
            << "i=" << to_string(i);
        EXPECT_NEAR(org->motion.vel.y(), sub->motion.vel.y(), 0.000001)
            << "i=" << to_string(i);
        EXPECT_NEAR(org->motion.pos.x(), sub->motion.pos.x(), 0.000001)
            << "i=" << to_string(i);
        EXPECT_NEAR(org->motion.pos.y(), sub->motion.pos.y(), 0.00001)
            << "i=" << to_string(i);
    }

    // Create 9 subPaths from the compositePaths
    vector<unique_ptr<Path>> subPaths;
    diff = 1000ms;
    for (auto i = 0ms; i < 8s; i += diff) {
        subPaths.push_back(compositePath.subPath(i, i + diff));
    }
    subPaths.push_back(compositePath.subPath(8000ms));

    // Compare the subPaths of the compositePaths to the origional path and
    // check that the results of evaluating the paths are close enough
    for (int i = 0; i < 9; i++) {
        for (auto j = 0ms; j < 1s; j += 100ms) {
            auto time = i * 1000ms + j;
            auto org = path.evaluate(time);
            auto sub = subPaths[i]->evaluate(j);
            ASSERT_TRUE(org);
            ASSERT_TRUE(sub);
            EXPECT_NEAR(org->motion.vel.x(), sub->motion.vel.x(), 0.000001)
                << "newPathTime=" << to_string(time);
            EXPECT_NEAR(org->motion.vel.y(), sub->motion.vel.y(), 0.000001)
                << "newPathTime=" << to_string(time);
            EXPECT_NEAR(org->motion.pos.x(), sub->motion.pos.x(), 0.000001)
                << "newPathTime=" << to_string(time);
            EXPECT_NEAR(org->motion.pos.y(), sub->motion.pos.y(), 0.00001)
                << "newPathTime=" << to_string(time);
        }
    }
}

}  // namespace Planning
