#include "Opponent.hpp"
#include "Robot.hpp"
#include "GameplayModule.hpp"

#include <Constants.hpp>

Gameplay::Opponent::Opponent(GameplayModule *gameplay, const std::string &name):
    _gameplay(gameplay),
    _name(name),
    _robot(0)
{
}

Gameplay::Opponent::~Opponent()
{
}

void Gameplay::Opponent::select()
{
    _robot = 0;
    float best_score = 0;
    for (int i = 0; i < 5; ++i)
    {
        Robot *r = _gameplay->opp[i];
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

float Gameplay::Opponent_Near_Ball::score(Robot *robot) const
{
    return (robot->state()->pos - _gameplay->state()->ball.pos).magsq();
}

////////

float Gameplay::Opponent_Near_Goal::score(Robot *robot) const
{
    return robot->state()->pos.magsq();
}
