#include "Orient.hpp"

#include <Sizes.h>
#include <Geometry/Point2d.hpp>

using namespace Geometry;

Tactics::Factory_Type<Tactics::Orient> orient("orient");

Tactics::Orient::Orient(Role *role) :
    Base(role), _pos_param(this, "pos")
{
}

float Tactics::Orient::score(Robot *robot)
{
    return robot->pos().distTo(vision_packet.ball.pos);
}

void Tactics::Orient::run()
{
	Packet::SkillCmd::Robot& skill = *robot()->skill();
    skill.valid = true;
    skill.skill = Packet::SkillCmd::Orient;
    skill.motion.face = _pos_param.point();
}

bool Tactics::Orient::done()
{
    return robot()->skill_status_code() == Packet::SkillStatus::Done;
}
