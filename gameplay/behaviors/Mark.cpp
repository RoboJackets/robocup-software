#include "Mark.hpp"
#include "../Role.hpp"

#include <Constants.hpp>
#include <LogFrame.hpp>
#include <vector>

#include <iostream>
using namespace std;
using namespace Geometry2d;

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Mark> behavior("mark");

Gameplay::Behaviors::Mark::Mark(GameplayModule *gameplay, Role *role):
	Behavior(gameplay, role),
	target_param(this,"target"),
	coverGoal_param(this,"coverGoal")
	{

	}

void Gameplay::Behaviors::Mark::run()
{
	//Copy in important state data
	const Packet::LogFrame::Ball& b = ball();
	Geometry2d::Point ball_pos = b.pos;
	Robot* opp = target_param.robot();
	float dist_to_man = Constants::Robot::Radius * 3;
	Point dest;

	//check whether marking for passes or a shooter
	if (coverGoal_param.valid() && coverGoal_param.value())
	{
		//stay between opp and goal
		Geometry2d::Segment goal_line(Point(Constants::Field::GoalWidth/2, 0),Point(-Constants::Field::GoalWidth/2, 0));
		Geometry2d::Point vel_correction = opp->vel()*0.5;
		Point opp_proj = opp->pos() + vel_correction *0.5;
		Point goal_point = goal_line.nearestPoint(opp_proj);
		dest = opp_proj - (opp->pos() - goal_point).normalized()*dist_to_man;
	}
	else
	{
		//Calculate position to obstruct path
		Geometry2d::Point static_dest = opp->pos() - (opp->pos()-ball_pos).normalized()*dist_to_man;
		Geometry2d::Point vel_correction = opp->vel()*0.5;
		//Project 1 sec ahead
		dest = static_dest + vel_correction * 0.5;

	}
	//go to destination
	robot()->move(dest);
	//always face ball
	robot()->face(ball_pos);
}



