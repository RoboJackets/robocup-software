#include "Move.hpp"
#include "../Role.hpp"

#include <LogFrame.hpp>

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Move> behavior("move");

Gameplay::Behaviors::Move::Move(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role),
    pos_param(this, "pos"),
    face_param(this, "face"),
    threshold_param(this, "threshold", 0.05),
    backoff_param(this, "backoff", 0)
{
}

void Gameplay::Behaviors::Move::run()
{
    Geometry2d::Point goal = pos_param.point();
    Geometry2d::Point cur = robot()->state()->pos;
    
    robot()->move(goal - (goal - cur).normalized() * backoff_param.value());
    
    if (face_param.valid())
    {
    	robot()->face(face_param.point());
    }
}

bool Gameplay::Behaviors::Move::done()
{
    const Packet::LogFrame::Robot *state = robot()->state();
    if (state->valid)
    {
        bool ret = state->pos.distTo(pos_param.point()) < threshold_param.value();
        return ret;
    } else {
        return false;
    }
}

float Gameplay::Behaviors::Move::score(Robot* robot)
{
	return robot->state()->pos.distTo(pos_param.point());
}
