#include "Skill.hpp"

using namespace Packet;

Skill::Skill(SkillCmd::Skill type) :
	_status(SkillStatus::None), _type(type)
{
	_robot = 0;
}

Skill::~Skill()
{
}
