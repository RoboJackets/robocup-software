#include "Steal.hpp"

#include <Geometry/Point2d.hpp>

using namespace Geometry;

Tactics::Factory_Type<Tactics::Steal> steal("steal");

Tactics::Steal::Steal(Role *role) :
    Base(role),
    _opp(this, "opp")
{
}

bool Tactics::Steal::done()
{
	return (robot()->skill_status_code() == Packet::SkillStatus::Done);
}

void Tactics::Steal::run()
{
	Packet::SkillCmd::Robot& skill = *robot()->skill();
	skill.valid = true;
	skill.skill = Packet::SkillCmd::StealBall;
	
	/** set position to opponent robot */
	skill.motion.pos = _opp.robot()->pos();
}
