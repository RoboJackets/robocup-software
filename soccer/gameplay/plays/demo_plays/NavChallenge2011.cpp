#include "NavChallenge2011.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::NavChallenge2011, "Demos")

Gameplay::Plays::NavChallenge2011::NavChallenge2011(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::NavChallenge2011::run()
{
	// assign a single robot (for now)

	// course setup: copied from diagram
	// starting from segments closest to goal
	const Geometry2d::Segment path1(Geometry2d::Point( 0.5, Field_Length/2.0 - 1.0),
														Geometry2d::Point(-2.025, Field_Length/2.0 - 1.0));
	const Geometry2d::Segment path2(Geometry2d::Point(-0.5, Field_Length/2.0 - 0.5),
														Geometry2d::Point( 2.025, Field_Length/2.0 - 0.5));
	const Geometry2d::Segment path3(Geometry2d::Point( 0.5, Field_Length/2.0 + 0.5),
														Geometry2d::Point(-2.025, Field_Length/2.0 + 0.5));
	const Geometry2d::Segment path4(Geometry2d::Point(-0.5, Field_Length/2.0 + 1.0),
														Geometry2d::Point( 2.025, Field_Length/2.0 + 1.0));

	gameplay()->state()->drawLine(path1);
	gameplay()->state()->drawLine(path2);
	gameplay()->state()->drawLine(path3);
	gameplay()->state()->drawLine(path4);

	// construct gated path through obstacles

	// construct course obstacles

	return true;
}
