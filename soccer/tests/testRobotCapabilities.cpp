#include <gtest/gtest.h>
#include <RobotCapabilities.hpp>

/* ************************************************************************* */
// tests for comparisons of robot capabilities and general inferface

/* ************************************************************************* */
TEST( testRobotCapabilities, basics ) {
	// default constructor - defaults to visible (useful for requirements)
	RobotCapbilities cap1;
	EXPECT_TRUE(cap1.visible());



}
