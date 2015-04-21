#include "TestFixture.hpp"

// add this to subclasses to run a test
//REGISTER_PLAY_CATEGORY(Gameplay::Plays::ExamplePlay, "Tests")

using namespace std;

Gameplay::Testing::TestFixture::TestFixture(GameplayModule *gameplay)
	: Play(gameplay)
{
}

bool Gameplay::Testing::TestFixture::run() {
	// TODO: create test environment and then execute test
	return false;
}


bool Gameplay::Testing::TestFixture::setupTest() {
	return false;
}

Gameplay::Testing::TestFixture::ResultSet Gameplay::Testing::TestFixture::excuteTest() {
	ResultSet ret;
	return ret;	// dummy return - nothing to report
}
