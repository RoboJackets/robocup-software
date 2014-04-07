
#include "gtest/gtest.h"
#include <Geometry2d/Point.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include <math.h>


bool trapezoid1(float t, float &posOut, float &speedOut) {
	return TrapezoidalMotion(10,	//	pathLength
		2,	//	maxSpeed
		1,	//	maxAcc
		t,	//	timeIntoLap
		0,	//	startSpeed
		0,	//	finalSpeed
		posOut,	//	&posOut
		speedOut);	//	&speedOut
}

TEST(TrapezoidalMotion, PreStart) {
	//	make sure it gives good values for negative t values
	float posOut, speedOut;
	bool pathValid = trapezoid1(-2, posOut, speedOut);
	EXPECT_EQ(pathValid, false);
	EXPECT_NEAR(posOut, 0, 0.001);
	EXPECT_NEAR(speedOut, 0, 0.001);
}

TEST(TrapezoidalMotion, Start) {
	//	beginning of the trapezoid, t = 0
	float posOut, speedOut;
	bool pathValid = trapezoid1(0, posOut, speedOut);
	EXPECT_NEAR(speedOut, 0, 0.001);
	EXPECT_NEAR(posOut, 0, 0.001);
	EXPECT_EQ(pathValid, true);
}

TEST(TrapezoidalMotion, End) {
	//	way after the trapezoid finishes
	float posOut, speedOut;
	bool pathValid = trapezoid1(50, posOut, speedOut);
	EXPECT_NEAR(speedOut, 0, 0.001) << "Speed should be zero at the end of the run";
	EXPECT_NEAR(posOut, 10, 0.001) << "Position should stay at end of path after path finishes";
	EXPECT_EQ(pathValid, false);
}


//	this path is too short for us to get to maxSpeed,
//	so it ends up being a triangular velocity profile,
//	rather than a trapezoid
bool triangle1(float t, float &posOut, float &speedOut) {
	return TrapezoidalMotion(2,	//	pathLength
		4,			//	maxSpeed
		0.5,		//	maxAcc
		t,			//	timeIntoLap
		0,			//	startSpeed
		0,			//	finalSpeed
		posOut,		//	&posOut
		speedOut);	//	&speedOut
}

TEST(TrapezoidalMotion, TriangleRampUp) {
	float posOut, speedOut;
	bool pathValid = triangle1(1, posOut, speedOut);
	EXPECT_EQ(pathValid, true);
	EXPECT_NEAR(posOut, 0.5*0.5, 0.001);
	EXPECT_NEAR(speedOut, 0.5, 0.001);
}
