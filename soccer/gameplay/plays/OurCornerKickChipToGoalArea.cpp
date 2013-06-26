/*
 * OurCornerKickChipToGoalArea.cpp
 *
 *  Created on: Jun 26, 2013
 *      Author: matt
 */

#include "OurCornerKickChipToGoalArea.hpp"
#include <boost/foreach.hpp>

namespace Gameplay {
namespace Plays {

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurCornerKick_ChipToGoalArea, "Restarts");

using namespace Geometry2d;
using namespace std;

OurCornerKick_ChipToGoalArea::OurCornerKick_ChipToGoalArea(GameplayModule* gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_center1(gameplay),
	_center2(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_pdt(gameplay, &_kicker)
{
	_fullback2.otherFullbacks.insert(&_fullback1);
	_fullback1.otherFullbacks.insert(&_fullback1);

	_center1.target = Point(0.0, Field_Length / 2.0);

	_target = Segment(Point(0,Field_Length-Field_PenaltyDist), Point(0,Field_Length));
}

float OurCornerKick_ChipToGoalArea::score(GameplayModule* gameplay)
{
	const GameState &gs = gameplay->state()->gameState;
		Point ballPos = gameplay->state()->ball.pos;
		bool chipper_available = false;
		BOOST_FOREACH(OurRobot * r, gameplay->playRobots())
		{
			if (r && r->chipper_available())
			{
				chipper_available = true;
				break;
			}
		}
		return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.0)) ? 1 : INFINITY;
}

bool OurCornerKick_ChipToGoalArea::run()
{
	set<OurRobot*> available = _gameplay->playRobots();

	if(!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		return false;
	}

	assignNearest(_center1.robot, available, Point(-Field_Width/4.0, Field_Length));
	assignNearest(_center2.robot, available, Point( Field_Width/4.0, Field_Length));

	// Set center targets
	if(ball().pos.x < 0)
	{
		_center1.target = Point(Field_ArcRadius + 0.5, Field_Length - Robot_Radius - 0.1);
		_center2.target = Point(Field_ArcRadius + 0.5, Field_Length - Robot_Radius - Robot_Diameter - 0.2);
	}
	else
	{
		_center1.target = Point(-Field_ArcRadius - 0.5, Field_Length - Robot_Radius - 0.1);
		_center2.target = Point(-Field_ArcRadius - 0.5, Field_Length - Robot_Radius - Robot_Diameter - 0.2);
	}
	_center1.face = ball().pos;
	_center2.face = ball().pos;

	state()->drawLine(_target.pt[0], _target.pt[1], QColor(255,255,0), "ChipToGoal");

	// Setup kicker
	_kicker.setTarget(_target);
	_kicker.use_chipper = true;
	_kicker.use_line_kick = false;

	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);
	assignNearest(_fullback1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_fullback2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	_pdt.run(); // calls _kicker.run()
	_center1.run();
	_center2.run();
	_fullback1.run();
	_fullback2.run();

	return _pdt.keepRunning();
}

OurCornerKick_ChipToGoalArea::~OurCornerKick_ChipToGoalArea() {
}

} /* namespace Plays */
} /* namespace Gameplay */
