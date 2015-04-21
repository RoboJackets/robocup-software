/*
 * RunAcrossTheField.cpp
 *
 *  Created on: Jun 24, 2013
 *      Author: matt
 */

#include "RunAcrossTheField.hpp"

namespace Gameplay {
namespace Plays {

REGISTER_PLAY_CATEGORY(Gameplay::Plays::RunAcrossTheField, "PID Tuning")

using namespace Geometry2d;
using namespace std;

RunAcrossTheField::RunAcrossTheField(GameplayModule* gameplay):
		Play(gameplay),
		_move(gameplay)
{
	_running = true;
	A = Point(-Field_Width/4.0,2.0);
	B = Point( Field_Width/4.0,2.0);
	T = A;
	_hasSetT = false;
}

bool RunAcrossTheField::run()
{
	if(_running)
	{
		set<OurRobot*> available = gameplay()->playRobots();
		assignNearest(_move.robot, available, Point(0,0));

		state()->drawCircle(A, Robot_Radius, QColor(255,0,255), "RATF");
		state()->drawCircle(B, Robot_Radius, QColor(255,0,255), "RATF");

		if(_move.robot)
		{
			if(!_hasSetT)
			{
				if(_move.robot->pos.distTo(A) < _move.robot->pos.distTo(B))
				{
					// destination is B
					T = B;
					_move.target = B;
					_move.robot->addText("Target: B", QColor(255,255,255), "RATF");
				}
				else
				{
					// destination is A
					T = A;
					_move.target = A;
					_move.robot->addText("Target: B", QColor(255,255,255), "RATF");
				}
				_hasSetT = true;
			}
			else
			{
				if(_move.robot->pos.distTo(T) < 0.001)
				{
					_running = false;
				}
			}
			_move.run();
		}

		return true;
	}
	return false;
}

RunAcrossTheField::~RunAcrossTheField() {
}

} /* namespace Plays */
} /* namespace Gameplay */
