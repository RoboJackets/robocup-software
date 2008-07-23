#include "Receive.hpp"

using namespace Packet;

Tactics::Factory_Type<Tactics::Receive> receive_tactic("receive");

Tactics::Receive::Receive(Role *role):
    Base(role)
{
}

void Tactics::Receive::start()
{
}

void Tactics::Receive::run()
{
    Packet::SkillCmd::Robot *skill = robot()->skill();
    
    skill->valid = true;
    skill->skill = Packet::SkillCmd::Receive;
}

bool Tactics::Receive::done()
{
    return robot()->skill_status_code() == Packet::SkillStatus::Done;
}
