#include "Move.hpp"
#include "../../Role.hpp"

using namespace Packet;

Tactics::Factory_Type<Tactics::Move> move_tactic("move");

Tactics::Move::Move(Role *role):
    Base(role),
    pos_param(this, "pos"),
    face_param(this, "face"),
    threshold_param(this, "threshold", 0.05)
{
}

void Tactics::Move::run()
{
    robot()->move(pos_param.point());
    
    if (face_param.valid())
    {
        robot()->face(face_param.point());
    }
}

bool Tactics::Move::done()
{
    const VisionData::Robot *vis = robot()->vision();
    if (vis->valid)
    {
        return vis->pos.distTo(pos_param.point()) < threshold_param.value();
    } else {
        return false;
    }
}

float Tactics::Move::score(Robot* robot)
{
	return robot->vision()->pos.distTo(pos_param.point());
}
