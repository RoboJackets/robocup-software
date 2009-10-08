#include "Example.hpp"

Gameplay::Behaviors::ChangeMe::ChangeMe(GameplayModule *gameplay):
    Behavior(gameplay)
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
