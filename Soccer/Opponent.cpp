#include "Opponent.hpp"
#include "Robot.hpp"

#include <Sizes.h>

Opponent::Opponent(const std::string &name):
    _name(name), _robot(0)
{
}

Opponent::~Opponent()
{
}

void Opponent::select()
{
    _robot = 0;
    float best_score = 0;
    for (int i = 0; i < 5; ++i)
    {
        Robot *r = &Robot::opp[i];
        float s = score(r);
        if (!r->assigned() && (!_robot || s < best_score))
        {
            _robot = r;
            best_score = s;
        }
    }
    
    if (_robot)
    {
        _robot->opponent(this);
    }
}

////////

float Opponent_Near_Ball::score(Robot *robot) const
{
    return (robot->vision()->pos - vision_packet.ball.pos).magsq();
}

////////

float Opponent_Near_Goal::score(Robot *robot) const
{
    return robot->vision()->pos.magsq();
}
