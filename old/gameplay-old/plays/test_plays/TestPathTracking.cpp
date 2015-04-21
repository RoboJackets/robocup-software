#include "TestPathTracking.hpp"

using namespace std;

// add this to subclasses to run a test
REGISTER_PLAY_CATEGORY(Gameplay::Testing::TestPathTracking, "Tests")

Gameplay::Testing::TestPathTracking::TestPathTracking(GameplayModule* gameplay)
	: TestFixture(gameplay)
{

}

/**
 * Sets the state so that there is only one robot on the field
 */
bool Gameplay::Testing::TestPathTracking::setupTest() {

	// remove all but one robot

	// wait until state catches up, then return true

	return false;
}

/**
 * Moves the robot through a path
 * TEST: robot gets to destination
 * TEST: robot does not deviate too much
 */
Gameplay::Testing::TestFixture::ResultSet
Gameplay::Testing::TestPathTracking::excuteTest() {


	ResultSet ret;
	return ret;
}
