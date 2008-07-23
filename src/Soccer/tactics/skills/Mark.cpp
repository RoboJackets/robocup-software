#include "Mark.hpp"
#include "../Role.hpp"

using namespace Packet;

Tactics::Factory_Type<Tactics::Mark> mark_tactic("mark");

Tactics::Mark::Mark(Role *role):
    Base(role),
    opp_param(this, "opp"),
    distance_param(this, "distance", 0.25)
{
}

void Tactics::Mark::run()
{
	SkillCmd::Robot& skill = *robot()->skill();

	if (opp_param.valid())
	{
		skill.valid = true;
		skill.markID = opp_param.robot()->id();
	}
	else
	{
		skill.valid = false;
	}
}
