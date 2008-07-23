#include "Mark.hpp"

#include <Geometry/Point2d.hpp>

#include "../Robot.hpp"

using namespace Packet;
using namespace Geometry;

Mark::Mark() :
	Skill(Packet::SkillCmd::Mark)
{
}

Packet::MotionCmd::Robot Mark::run()
{
	Packet::MotionCmd::Robot mCmd;
	mCmd.valid = true;
	
	//get opponent's id to mark
	const VisionData::Robot& opp = robot()->vision().opp[_cmd.markID];
	
	if (opp.valid)
	{
		Geometry::Point2d opp_pos = opp.pos;
		Geometry::Point2d facing = Point2d::direction(opp.theta * M_PI / 180.0);
		Geometry::Point2d our_pos = opp_pos + facing * .25; //distance needs to be changeable
		
		//TODO, distance should be variable based on the distance to ball
		
		mCmd.pos = our_pos;
		mCmd.face = our_pos + facing;
	}
	
	return mCmd;
}

