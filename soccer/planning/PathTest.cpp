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
    path.waypoints.emplace_back(MotionInstant(p0, Point()), 0);
    path.waypoints.emplace_back(MotionInstant(p1, Point()), 0);
    path.waypoints.emplace_back(MotionInstant(p2, Point()), 0);
    path.waypoints.emplace_back(MotionInstant(p3, Point()), 0);

    Segment actSeg = path.nearestSegment(Point(0.5, -0.5));
    EXPECT_FLOAT_EQ(p0.x, actSeg.pt[0].x);
    EXPECT_FLOAT_EQ(p0.y, actSeg.pt[0].y);
    EXPECT_FLOAT_EQ(p1.x, actSeg.pt[1].x);
    EXPECT_FLOAT_EQ(p1.y, actSeg.pt[1].y);
}

TEST(InterpolatedPath, evaluate) {
    Point p0(1, 1), p1(1, 2), p2(2, 2);

    InterpolatedPath path;
    path.waypoints.emplace_back(MotionInstant(p0, Point()), 0);
    path.waypoints.emplace_back(MotionInstant(p1, p1), 3);
    path.waypoints.emplace_back(MotionInstant(p2, Point()), 6);

    // path should be invalid and at end state when t > duration
    auto out = path.evaluate(1000);
    ASSERT_EQ(boost::none, out);
}

TEST(InterpolatedPath, subPath1) {
    InterpolatedPath path;
    path.waypoints.emplace_back(MotionInstant(Point(1, 1), Point(0, 0)), 0);
    path.waypoints.emplace_back(MotionInstant(Point(1, 2), Point(1, 1)), 3);
    path.waypoints.emplace_back(MotionInstant(Point(1, 3), Point(0, 0)), 6);

    //  test invalid parameters to subPath()
    EXPECT_THROW(path.subPath(-1, 5), invalid_argument);
    EXPECT_THROW(path.subPath(8, 20),
                 invalid_argument);  //  start time beyond bounds of path
    EXPECT_THROW(path.subPath(5, -2), invalid_argument);

    //  make a subpath that cuts off one second from the start and end of the
    //  original
    unique_ptr<Path> subPath = path.subPath(1, 5);
    float midTime = (5 - 1) / 2.0;
    auto mid = subPath->evaluate(midTime);
    ASSERT_NE(boost::none, mid);
    EXPECT_FLOAT_EQ(Point(1, 1).x, mid->motion.vel.x);

    //  mid velocity of subpath should be the same as velocity of original path
    EXPECT_FLOAT_EQ(Point(1, 1).y, mid->motion.vel.y);

    EXPECT_FLOAT_EQ(1, mid->motion.pos.x);
    EXPECT_FLOAT_EQ(2, mid->motion.pos.y);

    //  test the subpath at t = 0
    auto start = subPath->evaluate(0);
    ASSERT_NE(boost::none, start);

    //  the starting velocity of the subpath should be somewhere between the 0
    //  and the velocity at the middle
    EXPECT_GT(start->motion.vel.mag(), 0);
    EXPECT_LT(start->motion.vel.mag(), Point(1, 1).mag());

    //  the starting position of the subpath should be somewhere between the
    //  start pos of the original path and the middle point
    EXPECT_GT(start->motion.pos.y, 1);
    EXPECT_LT(start->motion.pos.x, 2);
}

TEST(InterpolatedPath, subpath2) {
    // Create a test path
    InterpolatedPath path;
    path.waypoints.emplace_back(MotionInstant(Point(1, 0), Point(0, 0)), 0);
    path.waypoints.emplace_back(MotionInstant(Point(1, 2), Point(-1, -1)), 1);
    path.waypoints.emplace_back(MotionInstant(Point(-2, 19), Point(1, 1)), 3);
    path.waypoints.emplace_back(MotionInstant(Point(1, 6), Point(0, 0)), 9);

    // Create 6 subPaths of length 1.5
    vector<unique_ptr<Path>> subPaths;
    float diff = 1.5;
    for (float i = 0; i < 9; i += diff) {
        subPaths.push_back(path.subPath(i, i + diff));
    }

    // Compare the subPaths to the origional path and check that the results of
    // evaluating the paths are close enough
    for (int i = 0; i < 6; i++) {
        for (float j = 0; j < 1.5; j += 0.001) {
            auto org = path.evaluate(i * 1.5 + j);
            auto sub = subPaths[i]->evaluate(j);

            ASSERT_NE(boost::none, org);
            ASSERT_NE(boost::none, sub);
            EXPECT_NEAR(org->motion.vel.x, sub->motion.vel.x, 0.000001)
                << "i+j=" << i + j;
            EXPECT_NEAR(org->motion.vel.y, sub->motion.vel.y, 0.000001)
                << "i+j=" << i + j;
            EXPECT_NEAR(org->motion.pos.x, sub->motion.pos.x, 0.000001)
                << "i+j=" << i + j;
            EXPECT_NEAR(org->motion.pos.y, sub->motion.pos.y, 0.00001) << "i+j="
                                                                       << i + j;
        }
    }
}

TEST(CompositePath, CompositeSubPath) {
    // Create a test path
    InterpolatedPath path;
    path.waypoints.emplace_back(MotionInstant(Point(1, 0), Point(0, 0)), 0);
    path.waypoints.emplace_back(MotionInstant(Point(1, 2), Point(-1, -1)), 1);
    path.waypoints.emplace_back(MotionInstant(Point(-2, 19), Point(1, 1)), 3);
    path.waypoints.emplace_back(MotionInstant(Point(1, 6), Point(0, 0)), 9);

    // Create 6 subPaths and rejoin them together into one compositePath
    CompositePath compositePath;
    float diff = 1.5;
    for (float i = 0; i < 7.5; i += diff) {
        compositePath.append(path.subPath(i, i + diff));
    }
    compositePath.append(path.subPath(7.5));

    // Compare that the compositePath and origonal path are mostly equal
    for (float i = 0; i <= 10; i += 0.001) {
        auto org = path.evaluate(i);
        auto sub = compositePath.evaluate(i);
        if (!org && !sub) break;

        ASSERT_NE(boost::none, org);
        ASSERT_NE(boost::none, sub);
        EXPECT_NEAR(org->motion.vel.x, sub->motion.vel.x, 0.000001) << "i="
                                                                    << i;
        EXPECT_NEAR(org->motion.vel.y, sub->motion.vel.y, 0.000001) << "i="
                                                                    << i;
        EXPECT_NEAR(org->motion.pos.x, sub->motion.pos.x, 0.000001) << "i="
                                                                    << i;
        EXPECT_NEAR(org->motion.pos.y, sub->motion.pos.y, 0.00001) << "i=" << i;
    }

    // Create 9 subPaths from the compositePaths
    vector<unique_ptr<Path>> subPaths;
    diff = 1;
    for (float i = 0; i < 8; i += diff) {
        subPaths.push_back(compositePath.subPath(i, i + diff));
    }
    subPaths.push_back(compositePath.subPath(8));

    // Compare the subPaths of the compositePaths to the origional path and
    // check that the results of evaluating the paths are close enough
    for (int i = 0; i < 9; i++) {
        for (float j = 0; j < 1; j += 0.1) {
            auto org = path.evaluate(i * 1 + j);
            auto sub = subPaths[i]->evaluate(j);
            ASSERT_NE(boost::none, org);
            ASSERT_NE(boost::none, sub);
            EXPECT_NEAR(org->motion.vel.x, sub->motion.vel.x, 0.000001)
                << "i+j=" << i + j;
            EXPECT_NEAR(org->motion.vel.y, sub->motion.vel.y, 0.000001)
                << "i+j=" << i + j;
            EXPECT_NEAR(org->motion.pos.x, sub->motion.pos.x, 0.000001)
                << "i+j=" << i + j;
            EXPECT_NEAR(org->motion.pos.y, sub->motion.pos.y, 0.00001) << "i+j="
                                                                       << i + j;
        }
    }
}

}  // namespace Planning
