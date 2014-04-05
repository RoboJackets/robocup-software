
#include "gtest/gtest.h"
#include <Geometry2d/Point.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include <math.h>


bool trapezoid1(float t, float &posOut, float speedOut) {
	return TrapezoidalMotion(10,	//	pathLength
		2,	//	maxSpeed
		1,	//	maxAcc
		t,	//	timeIntoLap
		0,	//	startSpeed
		0,	//	finalSpeed
		posOut,	//	&posOut
		speedOut);	//	&speedOut
}


TEST(TrapezoidalMotion, Speed) {
	float posOut, speedOut;

	//	beginning of the trapezoid, t = 0
	bool done = trapezoid1(0, posOut, speedOut);
	EXPECT_FLOAT_EQ(speedOut, 0) << "Speed should be zero at the beginning of the run";
	EXPECT_FLOAT_EQ(posOut, 0);
	EXPECT_EQ(done, false);

	//	way after the trapezoid finishes
	done = trapezoid1(50, posOut, speedOut);
	EXPECT_FLOAT_EQ(speedOut, 0) << "Speed should be zero at the end of the run";
	EXPECT_FLOAT_EQ(posOut, 10) << "Position should stay at end of path after path finishes";
	EXPECT_EQ(done, true);
}
