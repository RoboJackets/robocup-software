
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


//Triangular path of length 2
//startSpeed = 0, highSpeed = 1, endSpeed = 0, average = 0.5
//time = 4 seconds
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


//Triangle path of length 8
//startSpeed = 0, highSpeed = 2, endSpeed = 0
//time = 8 seconds
bool triangle2(float t, float &posOut, float &speedOut) {
	return TrapezoidalMotion(8,	//	pathLength
		4,			//	maxSpeed
		0.5,		//	maxAcc
		t,			//	timeIntoLap
		0,			//	startSpeed
		0,			//	finalSpeed
		posOut,		//	&posOut
		speedOut);	//	&speedOut
}

//Triangle path of length 6
//startSpeed = 1, highSpeed = 2, endSpeed = 1
//time = 4 seconds
bool triangle3(float t, float &posOut, float &speedOut) {
	return TrapezoidalMotion(6,	//	pathLength
		4,			//	maxSpeed
		0.5,		//	maxAcc
		t,			//	timeIntoLap
		1,			//	startSpeed
		1,			//	finalSpeed
		posOut,		//	&posOut
		speedOut);	//	&speedOut
}

//Triangle path of length 6
//startSpeed = 1, highSpeed = 2, endSpeed = 1
//time = 4 seconds
bool triangle4(float t, float &posOut, float &speedOut) {
	return TrapezoidalMotion(7,	//	pathLength
		4,			//	maxSpeed
		0.5,		//	maxAcc
		t,			//	timeIntoLap
		1,			//	startSpeed
		0,			//	finalSpeed
		posOut,		//	&posOut
		speedOut);	//	&speedOut
}

TEST(TrapezoidalMotion, TriangleRampUp) {
	float posOut, speedOut;
	bool pathValid = triangle1(1, posOut, speedOut);
	EXPECT_EQ(pathValid, true);
	EXPECT_NEAR(posOut, 0.5*0.5, 0.00001);
	EXPECT_NEAR(speedOut, 0.5, 0.00001);

	pathValid = triangle1(2, posOut, speedOut);
	EXPECT_EQ(pathValid, true);
	EXPECT_NEAR(posOut,  1, 0.00001);
	EXPECT_NEAR(speedOut, 1, 0.00001);

	pathValid = triangle1(3, posOut, speedOut);
	EXPECT_EQ(pathValid, true);
	EXPECT_NEAR(posOut, 2 - 0.5*0.5, 0.00001);
	EXPECT_NEAR(speedOut, 0.5, 0.00001);
}

TEST(TrapezoidalMotion2, TriangleRampUp) {
	bool pathValid;
	float posOut, speedOut;
	for (float i =0; i<4; i+=0.01)
	{
		pathValid = triangle2(i, posOut, speedOut);
		ASSERT_EQ(pathValid, true);
		ASSERT_NEAR(speedOut, i/2, 0.00001);
	}

	pathValid = triangle2(4, posOut, speedOut);
	EXPECT_EQ(pathValid, true);
	EXPECT_NEAR(posOut, 4, 0.00001);
	EXPECT_NEAR(speedOut, 2, 0.00001);

	for (float i = 0; i<=4; i+=0.01)
	{
		pathValid = triangle2(i + 4, posOut, speedOut);
		ASSERT_EQ(pathValid, true);
		ASSERT_NEAR(speedOut, 2 - i/2, 0.00001);
	}
}

TEST(TrapezoidalMotion3, TriangleRampUp) {
	for (float i=0; i<4; i+=0.01) {
		float posOut2, speedOut2, posOut3, speedOut3;
		bool pathValid2 = triangle2(2 + i, posOut2, speedOut2);
		bool pathValid3 = triangle3(i, posOut3, speedOut3);
		ASSERT_EQ(pathValid2, pathValid3);
		ASSERT_NEAR(posOut2, posOut3 + 1, 0.00001);
		ASSERT_NEAR(speedOut2, speedOut3, 0.00001);
	}

	for (float i=0; i<6; i+=0.01) {
		float posOut2, speedOut2, posOut4, speedOut4;
		bool pathValid2 = triangle2(2 + i, posOut2, speedOut2);
		bool pathValid4 = triangle4(i, posOut4, speedOut4);
		ASSERT_EQ(pathValid2, pathValid4);
		ASSERT_NEAR(posOut2, posOut4 + 1, 0.00001);
		ASSERT_NEAR(speedOut2, speedOut4, 0.00001);
	}
}

