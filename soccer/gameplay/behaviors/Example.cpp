#include "Example.hpp"

#include "../Role.hpp"

static Gameplay::BehaviorFactoryType<Gameplay::Behaviors::ChangeMe> behavior("changeme");

Gameplay::Behaviors::ChangeMe::ChangeMe(GameplayModule *gameplay, Role *role):
    Behavior(gameplay, role),
    pos_param(this, "pos")
{
}

void Gameplay::Behaviors::ChangeMe::run()
{
    //...
}

bool Gameplay::Behaviors::ChangeMe::done()
{
    //...
}

float Gameplay::Behaviors::ChangeMe::score(Robot* robot)
{
	//...
}
