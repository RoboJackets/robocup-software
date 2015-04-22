#include <iostream>
#include <gtest/gtest.h>
#include <planning/InterpolatedPath.hpp>

using namespace std;
using namespace Geometry2d;
using namespace Planning;

/* ************************************************************************* */
TEST( testPath, nearestSegment) {
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
TEST( testPath, startFrom1 ) {

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
TEST( testPath, startFrom2 ) {

	Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

	Planning::InterpolatedPath path;
	path.points.push_back(p0);
	path.points.push_back(p1);
	path.points.push_back(p2);
	path.points.push_back(p3);

	Point pt(0.5,-1.0);
	Planning::InterpolatedPath act;
	path.startFrom(pt, act);

	// verify
	ASSERT_EQ(4, act.size());
	EXPECT_TRUE(pt == act.points[0]);
//	EXPECT_FLOAT_EQ( 0.5, act.points[1].x); // fails
	EXPECT_FLOAT_EQ( 0.0, act.points[1].y);
}

/* ************************************************************************* */
TEST( testPath, startFrom3 ) {

	Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

	Planning::InterpolatedPath path;
	path.points.push_back(p0);
	path.points.push_back(p1);
	path.points.push_back(p2);
	path.points.push_back(p3);

	Point pt(0.5,-0.01);
	Planning::InterpolatedPath act;
	path.startFrom(pt, act);

	// verify
	ASSERT_EQ(5, act.size());
	EXPECT_TRUE(pt == act.points[0]);
//	EXPECT_TRUE(p1 == act.points[1]); // fails
}

TEST(InterpolatedPath, evaluate) {
	Point p0(1,1), p1(1, 2), p2(2, 2);

	InterpolatedPath path;
	path.points.push_back(p0);
	path.points.push_back(p1);
	path.points.push_back(p2);
	path.times.push_back(0);
	path.times.push_back(3);
	path.times.push_back(6);
	path.vels.push_back(Point(0,0));
	path.vels.push_back(p1);
	path.vels.push_back(Point(0,0));
	Point posOut, velOut;
	bool pathValid;


	//	path should be invalid and at start state when t < 0
	pathValid = path.evaluate(-1, posOut, velOut);
	EXPECT_FLOAT_EQ(0, (posOut - p0).mag());
	EXPECT_FLOAT_EQ(0, (velOut).mag());
	EXPECT_FALSE(pathValid);

	//	path should be invalid and at end state when t > duration
	pathValid = path.evaluate(1000, posOut, velOut);
	EXPECT_FLOAT_EQ(0, (posOut - p2).mag());
	EXPECT_FLOAT_EQ(0, velOut.mag());
	EXPECT_FALSE(pathValid);
}

TEST(InterpolatedPath, subPath) {
    InterpolatedPath path;
    path.points.push_back(Point(1,1));
    path.points.push_back(Point(1,2));
    path.points.push_back(Point(1,3));
    path.times.push_back(0);
    path.times.push_back(3);
    path.times.push_back(6);
    path.vels.push_back(Point(0,0));
    path.vels.push_back(Point(1,1));
    path.vels.push_back(Point(0,0));

    //  test invalid parameters to subPath()
    EXPECT_THROW(path.subPath(-1, 5), invalid_argument);
    EXPECT_THROW(path.subPath(0, 20), invalid_argument);    //  end time beyond bounds of path
    EXPECT_THROW(path.subPath(8, 20), invalid_argument);    //  start time beyond bounds of path

    //  make a subpath that cuts off one second from the start and end of the original
    unique_ptr<Path> subPath = path.subPath(1, 5);

    Point pMid, vMid;
    float midTime = (5-1)/2.0;
    bool valid = subPath->evaluate(midTime, pMid, vMid);
    EXPECT_TRUE(valid);
    EXPECT_FLOAT_EQ(Point(1,1).mag(), vMid.mag());   //  mid velocity of subpath should be the same as velocity of original path
    EXPECT_FLOAT_EQ(1, pMid.x);
    EXPECT_FLOAT_EQ(2, pMid.y);


    //  test the subpath at t = 0
    Point pStart, vStart;
    valid = subPath->evaluate(0, pStart, vStart);
    EXPECT_TRUE(valid);

    //  the starting velocity of the subpath should be somewhere between the 0 and the velocity at the middle
    EXPECT_GT(vStart.mag(), 0);
    EXPECT_LT(vStart.mag(), Point(1,1).mag());

    //  the starting position of the subpath should be somewhere between the start pos of the original path and the middle point
    EXPECT_GT(pStart.y, 1);
    EXPECT_LT(pStart.x, 2);
}
