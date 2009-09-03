#include "Pivot.hpp"

#include "../../Role.hpp"

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Pivot> behavior("pivot");

using namespace Packet;

Gameplay::Behaviors::Pivot::Pivot(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role)
{
	_state = ApproachFast;
}

void Gameplay::Behaviors::Pivot::stop()
{
	_state = ApproachFast;
}

void Gameplay::Behaviors::Pivot::run()
{
	LogFrame::Robot& self = *robot()->state();
	const Packet::LogFrame::Ball& b = ball();
	
	//nothing really useful to do if no ball
	//will think of something :) - Roman
	if (!b.valid && !self.haveBall)
	{
		return;
	}
	
	if (_state == ApproachFast || _state == ApproachSlow)
	{		
		self.cmd.pivot = LogFrame::MotionCmd::NoPivot;
		
		float distFromBall = Constants::Robot::Radius;
		robot()->willKick = true;	
		
		self.radioTx.roller = 50;
		
		if (_state == ApproachFast)
		{
			distFromBall = Constants::Robot::Diameter;
			robot()->willKick = false;
			self.radioTx.roller = 0;
		}
		
		Geometry2d::Point pos = self.pos - b.pos;
		pos = pos.normalized() * distFromBall + b.pos;
		
		robot()->move(pos);
		robot()->face(b.pos);
		
		if (pos.nearPoint(self.pos, .03))
		{
			_pp = b.pos;
			
			if (_state == ApproachFast)
			{
				_state = ApproachSlow;
			}
			else
			{
				_state = PivotAround;
			}
		}
	}
	else if (_state == PivotAround)
	{
		if (!self.haveBall && !b.pos.nearPoint(self.pos, Constants::Robot::Radius + .05))
		{
			_state = ApproachFast;
		}
		
		self.cmd.pivotPoint = _pp;
		self.cmd.pivot = LogFrame::MotionCmd::CW;
		self.radioTx.roller = 20;
	}
}

bool Gameplay::Behaviors::Pivot::done()
{
    return false;
}
