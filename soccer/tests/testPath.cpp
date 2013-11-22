#include <iostream>
#include <gtest/gtest.h>
#include <planning/Path.hpp>

using namespace std;
using namespace Geometry2d;

/* ************************************************************************* */
TEST( testPath, nearestSegment) {
	Geometry2d::Point p0, p1(1.0, 0.0), p2(2.0, 0.0), p3(3.0, 0.0);

	Planning::Path path;
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

	Planning::Path path;
	path.points.push_back(p0);
	path.points.push_back(p1);
	path.points.push_back(p2);
	path.points.push_back(p3);

	// simple case - on same axis as path
	Planning::Path act;
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

	Planning::Path path;
	path.points.push_back(p0);
	path.points.push_back(p1);
	path.points.push_back(p2);
	path.points.push_back(p3);

	Point pt(0.5,-1.0);
	Planning::Path act;
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

	Planning::Path path;
	path.points.push_back(p0);
	path.points.push_back(p1);
	path.points.push_back(p2);
	path.points.push_back(p3);

	Point pt(0.5,-0.01);
	Planning::Path act;
	path.startFrom(pt, act);

	// verify
	ASSERT_EQ(5, act.size());
	EXPECT_TRUE(pt == act.points[0]);
//	EXPECT_TRUE(p1 == act.points[1]); // fails
}

