#include "Motion.hpp"
#include "structures.hpp"
#include "WorldModel.hpp"
#include "Radio.hpp"

Motion::Motion()
{

}

//Link to modules
void Motion::setWM(WorldModel * world)
{
  this->wm = world;
}

void  Motion::setRadio(Radio *rad)
{

}


void Motion::updateFromSoccer(state_pv * world, command_set * cmd)
{
  //Process data from world model and commands


  //Send controls to radio

  //Send controls to world model

}

