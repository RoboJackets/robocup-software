#include "Dribble.hpp"

#include "../Role.hpp"

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::Dribble> behavior("dribble");

Gameplay::Behaviors::Dribble::Dribble(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role),
    _dest(this, "dest"),
    _intercept(gameplay, role)
{
	_intercept.target_param.set(_dest.point());
}

void Gameplay::Behaviors::Dribble::start()
{
	_state = AcquireBall;
	_intercept.robot(robot());
}

void Gameplay::Behaviors::Dribble::run()
{
	if (!ball().valid)
	{
		return;
	}
	
    if (_state == AcquireBall)
    {
    	_intercept.run();
    	
    	if (_intercept.done())
    	{
    		_state = Translate;
    	}
    }
    
    if (_state == Translate)
    {
    	robot()->state()->cmd.vScale = .5;
    	
    	robot()->move(_dest.point());
    	robot()->face(ball().pos);
    }
}

bool Gameplay::Behaviors::Dribble::done()
{
    //...
	return false;
}

float Gameplay::Behaviors::Dribble::score(Robot* robot)
{
	//...
	return 0;
}
