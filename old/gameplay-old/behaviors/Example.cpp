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
