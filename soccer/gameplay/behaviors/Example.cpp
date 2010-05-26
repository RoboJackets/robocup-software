#include "Example.hpp"

Gameplay::Behaviors::ChangeMe::ChangeMe(GameplayModule *gameplay):
    Behavior(gameplay)
{
}

bool Gameplay::Behaviors::ChangeMe::run()
{
    //...
	return true;
}

bool Gameplay::Behaviors::ChangeMe::done()
{
    //...
}

float Gameplay::Behaviors::ChangeMe::score(Robot* robot)
{
	//...
}
