#include <iostream>
#include <gtest/gtest.h>
#include <planning/InterpolatedPath.hpp>
#include <planning/CompositePath.hpp>

using namespace std;
using namespace Geometry2d;
using namespace Planning;

/* ************************************************************************* */
TEST(Path, nearestSegment) {
    Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

    Planning::InterpolatedPath path;
    path.points.push_back(p0);
    path.points.push_back(p1);
    path.points.push_back(p2);
    path.points.push_back(p3);

    Segment actSeg = path.nearestSegment(Point(0.5, -0.5));
    EXPECT_FLOAT_EQ(p0.x, actSeg.pt[0].x);
    EXPECT_FLOAT_EQ(p0.y, actSeg.pt[0].y);
    EXPECT_FLOAT_EQ(p1.x, actSeg.pt[1].x);
    EXPECT_FLOAT_EQ(p1.y, actSeg.pt[1].y);
}

/* ************************************************************************* */
TEST(Path, startFrom1) {
    Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

    Planning::InterpolatedPath path;
    path.points.push_back(p0);
    path.points.push_back(p1);
    path.points.push_back(p2);
    path.points.push_back(p3);

    // simple case - on same axis as path
    Planning::InterpolatedPath act;
    path.startFrom(Point(-1.0, 0.0), act);

    // verify
    ASSERT_EQ(4, act.size());
    EXPECT_FLOAT_EQ(-1.0, act.points[0].x);
    EXPECT_FLOAT_EQ(0.0, act.points[0].y);
    EXPECT_TRUE(act.points[1] == p1);
}

/* ************************************************************************* */
TEST(Path, startFrom2) {
    Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

    Planning::InterpolatedPath path;
    path.points.push_back(p0);
    path.points.push_back(p1);
    path.points.push_back(p2);
    path.points.push_back(p3);

    Point pt(0.5, -1.0);
    Planning::InterpolatedPath act;
    path.startFrom(pt, act);

    // verify
    ASSERT_EQ(4, act.size());
    EXPECT_TRUE(pt == act.points[0]);
    //	EXPECT_FLOAT_EQ( 0.5, act.points[1].x); // fails
    EXPECT_FLOAT_EQ(0.0, act.points[1].y);
}

/* ************************************************************************* */
TEST(Path, startFrom3) {
    Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

    Planning::InterpolatedPath path;
    path.points.push_back(p0);
    path.points.push_back(p1);
    path.points.push_back(p2);
    path.points.push_back(p3);

    Point pt(0.5, -0.01);
    Planning::InterpolatedPath act;
    path.startFrom(pt, act);

    // verify
    ASSERT_EQ(5, act.size());
    EXPECT_TRUE(pt == act.points[0]);
    //	EXPECT_TRUE(p1 == act.points[1]); // fails
}

TEST(InterpolatedPath, evaluate) {
    Point p0(1, 1), p1(1, 2), p2(2, 2);

    InterpolatedPath path;
    path.points.push_back(p0);
    path.points.push_back(p1);
    path.points.push_back(p2);
    path.times.push_back(0);
    path.times.push_back(3);
    path.times.push_back(6);
    path.vels.push_back(Point(0, 0));
    path.vels.push_back(p1);
    path.vels.push_back(Point(0, 0));

    MotionInstant out;
    bool pathValid;

    //	path should be invalid and at end state when t > duration
    pathValid = path.evaluate(1000, out);
    EXPECT_FLOAT_EQ(0, (out.pos - p2).mag());
    EXPECT_FLOAT_EQ(0, out.vel.mag());
    EXPECT_FALSE(pathValid);
}

TEST(InterpolatedPath, subPath1) {
    InterpolatedPath path;
    path.points.push_back(Point(1, 1));
    path.points.push_back(Point(1, 2));
    path.points.push_back(Point(1, 3));
    path.times.push_back(0);
    path.times.push_back(3);
    path.times.push_back(6);
    path.vels.push_back(Point(0, 0));
    path.vels.push_back(Point(1, 1));
    path.vels.push_back(Point(0, 0));

    //  test invalid parameters to subPath()
    EXPECT_THROW(path.subPath(-1, 5), invalid_argument);
    EXPECT_THROW(path.subPath(8, 20),
                 invalid_argument);  //  start time beyond bounds of path
    EXPECT_THROW(path.subPath(5, -2), invalid_argument);

    //  make a subpath that cuts off one second from the start and end of the
    //  original
    unique_ptr<Path> subPath = path.subPath(1, 5);
    MotionInstant mid;
    float midTime = (5 - 1) / 2.0;
    bool valid = subPath->evaluate(midTime, mid);
    EXPECT_TRUE(valid);
    EXPECT_FLOAT_EQ(Point(1, 1).x, mid.vel.x);
    EXPECT_FLOAT_EQ(Point(1, 1).y, mid.vel.y);  //  mid velocity of subpath
                                                //  should be the same as
                                                //  velocity of original path
    EXPECT_FLOAT_EQ(1, mid.pos.x);
    EXPECT_FLOAT_EQ(2, mid.pos.y);

    //  test the subpath at t = 0
    MotionInstant start;
    valid = subPath->evaluate(0, start);
    EXPECT_TRUE(valid);

    //  the starting velocity of the subpath should be somewhere between the 0
    //  and the velocity at the middle
    EXPECT_GT(start.vel.mag(), 0);
    EXPECT_LT(start.vel.mag(), Point(1, 1).mag());

    //  the starting position of the subpath should be somewhere between the
    //  start pos of the original path and the middle point
    EXPECT_GT(start.pos.y, 1);
    EXPECT_LT(start.pos.x, 2);
}

TEST(InterpolatedPath, subpath2) {
    // Create a test path
    InterpolatedPath path;
    path.points.push_back(Point(1, 0));
    path.points.push_back(Point(1, 2));
    path.points.push_back(Point(-2, 19));
    path.points.push_back(Point(1, 6));
    path.times.push_back(0);
    path.times.push_back(1);
    path.times.push_back(3);
    path.times.push_back(9);
    path.vels.push_back(Point(0, 0));
    path.vels.push_back(Point(-1, -1));
    path.vels.push_back(Point(1, 1));
    path.vels.push_back(Point(0, 0));

    // Create 6 subPaths of length 1.5
    vector<unique_ptr<Path>> subPaths;
    float diff = 1.5;
    for (float i = 0; i < 9; i += diff) {
        subPaths.push_back(path.subPath(i, i + diff));
    }

    // Compare the subPaths to the origional path and check that the results of
    // evaluating the paths are close enough
    MotionInstant org, sub;
    for (int i = 0; i < 6; i++) {
        for (float j = 0; j < 1.5; j += 0.001) {
            bool validOrg = path.evaluate(i * 1.5 + j, org);
            bool validSub = subPaths[i]->evaluate(j, sub);
            EXPECT_NEAR(org.vel.x, sub.vel.x, 0.000001) << "i+j=" << i + j;
            EXPECT_NEAR(org.vel.y, sub.vel.y, 0.000001) << "i+j=" << i + j;
            EXPECT_NEAR(org.pos.x, sub.pos.x, 0.000001) << "i+j=" << i + j;
            EXPECT_NEAR(org.pos.y, sub.pos.y, 0.00001) << "i+j=" << i + j;
            EXPECT_EQ(validOrg, validSub) << "i+j=" << i + j;
        }
    }
}

TEST(CompositePath, CompositeSubPath) {
    // Create a test path
    InterpolatedPath path;
    path.points.push_back(Point(1, 0));
    path.points.push_back(Point(1, 2));
    path.points.push_back(Point(-2, 19));
    path.points.push_back(Point(1, 6));
    path.times.push_back(0);
    path.times.push_back(1);
    path.times.push_back(3);
    path.times.push_back(9);
    path.vels.push_back(Point(0, 0));
    path.vels.push_back(Point(-1, -1));
    path.vels.push_back(Point(1, 1));
    path.vels.push_back(Point(0, 0));

    // Create 6 subPaths and rejoin them together into one compositePath
    CompositePath compositePath;
    float diff = 1.5;
    for (float i = 0; i < 7.5; i += diff) {
        compositePath.append(path.subPath(i, i + diff));
    }
    compositePath.append(path.subPath(7.5));

    // Compare that the compositePath and origional path are mostly equal
    for (float i = 0; i <= 10; i += 0.001) {
        MotionInstant org, sub;
        bool validOrg = path.evaluate(i, org);
        bool validSub = compositePath.evaluate(i, sub);
        EXPECT_NEAR(org.vel.x, sub.vel.x, 0.000001) << "i=" << i;
        EXPECT_NEAR(org.vel.y, sub.vel.y, 0.000001) << "i=" << i;
        EXPECT_NEAR(org.pos.x, sub.pos.x, 0.000001) << "i=" << i;
        EXPECT_NEAR(org.pos.y, sub.pos.y, 0.00001) << "i=" << i;
        EXPECT_EQ(validOrg, validSub) << "i=" << i;
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
            MotionInstant org, sub;
            bool validOrg = path.evaluate(i * 1 + j, org);
            bool validSub = subPaths[i]->evaluate(j, sub);
            EXPECT_NEAR(org.vel.x, sub.vel.x, 0.000001) << "i+j=" << i + j;
            EXPECT_NEAR(org.vel.y, sub.vel.y, 0.000001) << "i+j=" << i + j;
            EXPECT_NEAR(org.pos.x, sub.pos.x, 0.000001) << "i+j=" << i + j;
            EXPECT_NEAR(org.pos.y, sub.pos.y, 0.00001) << "i+j=" << i + j;
            EXPECT_EQ(validOrg, validSub) << "i+j=" << i + j;
        }
    }
}
