#include "Behavior.hpp"



using namespace std;

Gameplay::Behavior::Behavior(GameplayModule *gameplay):
_gameplay(gameplay)
{
}

Gameplay::Behavior::~Behavior()
{
}

bool Gameplay::Behavior::run()
{
	return false;
}
