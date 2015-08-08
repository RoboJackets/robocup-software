
#include "gtest/gtest.h"
#include <Geometry2d/Point.hpp>
#include <motion/TrapezoidalMotion.hpp>
#include <math.h>
#include <iostream>

using namespace std;

bool trapezoid1(float t, float &posOut, float &speedOut) {
	float pathLength = 10;
	float maxSpeed = 2;
	float maxAcc = 1;
	float startSpeed = 0;
	float finalSpeed = 0;

	bool valid = TrapezoidalMotion(pathLength, maxSpeed, maxAcc, t,	startSpeed,	finalSpeed,	posOut,	speedOut);

	if (valid) {
		float time = Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed);
		EXPECT_NEAR(time, t, 0.001);
	}
	return valid;
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

bool trapezoid2(float t, float &posOut, float &speedOut) {
	float pathLength = 9.5;
	float maxSpeed = 2;
	float maxAcc = 1;
	float startSpeed = 1;
	float finalSpeed = 0;

	bool valid = TrapezoidalMotion(pathLength, maxSpeed, maxAcc, t,	startSpeed,	finalSpeed,	posOut,	speedOut);

	if (valid) {
		float time = Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed);
		EXPECT_NEAR(time, t, 0.001);
	}
	return valid;
}

TEST(TrapezoidalMotion2, MoreTests) {
	for (float i=0; i<6; i+=0.01) {
		float posOut1, speedOut1, posOut2, speedOut2;
		bool pathValid1 = trapezoid1(i+1, posOut1, speedOut1);
		bool pathValid2 = trapezoid2(i, posOut2, speedOut2);
		ASSERT_EQ(pathValid1, pathValid2);
		ASSERT_NEAR(posOut1, posOut2+0.5, 0.00001);
		ASSERT_NEAR(speedOut1, speedOut2, 0.00001);
	}
}

TEST(TrapezoidalMotion3, MoreTests) {
	float posOut = 1.11101;
	float pathLength = 1.11101;
	float maxSpeed = 2.2;
	float maxAcc = 1;
	float startSpeed = 0.901393;
	float finalSpeed = 0;

	float result = Trapezoidal::getTime(2.03294, 2.03294, 2.2, 1, 0.176091, 0);
	cout<<result<<endl;
	EXPECT_FALSE(isnan(result));
	cout<<Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed) << endl;
}

//Triangular path of length 2
//startSpeed = 0, highSpeed = 1, endSpeed = 0, average = 0.5
//time = 4 seconds
bool triangle1(float t, float &posOut, float &speedOut) {
	float pathLength = 2;
	float maxSpeed = 4;
	float maxAcc = 0.5;
	float startSpeed = 0;
	float finalSpeed = 0;

	bool valid = TrapezoidalMotion(pathLength, maxSpeed, maxAcc, t,	startSpeed,	finalSpeed,	posOut,	speedOut);

	if (valid) {
		float time = Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed);
		EXPECT_NEAR(time, t, 0.001);
	}
	return valid;
}


//Triangle path of length 8
//startSpeed = 0, highSpeed = 2, endSpeed = 0
//time = 8 seconds
bool triangle2(float t, float &posOut, float &speedOut) {
	float pathLength = 8;
	float maxSpeed = 4;
	float maxAcc = 0.5;
	float startSpeed = 0;
	float finalSpeed = 0;

	bool valid = TrapezoidalMotion(pathLength, maxSpeed, maxAcc, t,	startSpeed,	finalSpeed,	posOut,	speedOut);

	if (valid) {
		float time = Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed);
		EXPECT_NEAR(time, t, 0.001);
	}
	return valid;
}

//Triangle path of length 6
//startSpeed = 1, highSpeed = 2, endSpeed = 1
//time = 4 seconds
bool triangle3(float t, float &posOut, float &speedOut) {
	float pathLength = 6;
	float maxSpeed = 4;
	float maxAcc = 0.5;
	float startSpeed = 1;
	float finalSpeed = 1;

	bool valid = TrapezoidalMotion(pathLength, maxSpeed, maxAcc, t,	startSpeed,	finalSpeed,	posOut,	speedOut);

	if (valid) {
		float time = Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed);
		EXPECT_NEAR(time, t, 0.001);
	}
	return valid;
}

//Triangle path of length 6
//startSpeed = 1, highSpeed = 2, endSpeed = 1
//time = 4 seconds
bool triangle4(float t, float &posOut, float &speedOut) {
	float pathLength = 7;
	float maxSpeed = 4;
	float maxAcc = 0.5;
	float startSpeed = 1;
	float finalSpeed = 0;

	bool valid = TrapezoidalMotion(pathLength, maxSpeed, maxAcc, t,	startSpeed,	finalSpeed,	posOut,	speedOut);

	if (valid) {
		float time = Trapezoidal::getTime(posOut, pathLength, maxSpeed, maxAcc, startSpeed, finalSpeed);
		EXPECT_NEAR(time, t, 0.001);
	}
	return valid;
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

	pathValid = triangle1(4, posOut, speedOut);
	EXPECT_EQ(pathValid, false);
	EXPECT_NEAR(posOut, 2, 0.00001);
	EXPECT_NEAR(speedOut, 0, 0.00001);
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

	float i = 4;
	pathValid = triangle2(i + 4, posOut, speedOut);
	ASSERT_EQ(pathValid, false);
	ASSERT_NEAR(speedOut, 2 - i/2, 0.00001);
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

	float i = 6;
	float posOut2, speedOut2, posOut4, speedOut4;
	bool pathValid2 = triangle2(2 + i, posOut2, speedOut2);
	bool pathValid4 = triangle4(i, posOut4, speedOut4);
	//ASSERT_EQ(pathValid2, pathValid4);
	ASSERT_NEAR(posOut2, posOut4 + 1, 0.00001);
	ASSERT_NEAR(speedOut2, speedOut4, 0.00001);
}

