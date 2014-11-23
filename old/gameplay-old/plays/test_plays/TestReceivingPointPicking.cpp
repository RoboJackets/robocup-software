/*
 * TestPivotKick.cpp
 *
 *  Created on: Nov 11, 2012
 *      Author: Matthew Barulic
 */

#include "TestReceivingPointPicking.hpp"
#include <iterator>

#include <QtGui>

using namespace std;
using namespace Geometry2d;

namespace Gameplay {
namespace Plays {

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestReceivingPointPicking, "TestReceivingPointPicker")

TestReceivingPointPicking::TestReceivingPointPicking(GameplayModule *gameplay):
	Play(gameplay),
	_gameplay(gameplay),
	_move(gameplay)
{
}

bool TestReceivingPointPicking::run() {
	std::cout << "get robots" << std::endl;
	set<OurRobot *> available = _gameplay->playRobots();
	std::cout << "assign robot" << std::endl;
	assignNearest(_move.robot, available, Geometry2d::Point());

	if(_move.robot)
	{
		std::cout << "got a robot" << std::endl;
		Point A(1.0125, 0);
		Point B(1.0125, 5.05);
		Segment line = Segment(A, B);
		Point reveivePoint = ReceivePointEvaluator::FindReceivingPoint(_gameplay->state(), _move.robot->pos, ball().pos, line);
		std::cout << "got a point" << std::endl;
		state()->drawCircle(reveivePoint, Robot_Radius, QColor(255,0,0), "Receiver");
		std::cout << "drew the point" << std::endl;
	}

	return true;
}

TestReceivingPointPicking::~TestReceivingPointPicking() {
}

} /* namespace Plays */
} /* namespace Gameplay */
