#include <gtest/gtest.h>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/Line.hpp>

/* ************************************************************************* */
// Unit testing example - create additional tests for new features
// these can serve as examples

/* ************************************************************************* */
TEST( testExamples, test1 ) {
	Geometry2d::Point testPt(1.0,2.0);
	EXPECT_FLOAT_EQ(1.0, testPt.x);
	EXPECT_FLOAT_EQ(2.0, testPt.y);
}

/* ************************************************************************* */
TEST( testExamples, test2 ) {
	Geometry2d::Line ln(Geometry2d::Point(1.0, 2.0), Geometry2d::Point(5.0, 2.0));
	EXPECT_FLOAT_EQ(1.0, ln.pt[0].x);
	EXPECT_FLOAT_EQ(2.0, ln.pt[0].y);
	EXPECT_FLOAT_EQ(5.0, ln.pt[1].x);
	EXPECT_FLOAT_EQ(2.0, ln.pt[1].y);
}
