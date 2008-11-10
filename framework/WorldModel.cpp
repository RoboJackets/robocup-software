#include "WorldModel.hpp"

#include "Soccer.hpp"
#include "structures.hpp"

WorldModel::WorldModel()
{

}

//Links to other modules:
void WorldModel::setSoc(Soccer * s)
{
  this->soc = s;
}

//Update due to inputs
void WorldModel::updateFromInput(const state_p * state)
{

}

void WorldModel::updateFromMotion(const robot_control_set * cmds)
{

}

