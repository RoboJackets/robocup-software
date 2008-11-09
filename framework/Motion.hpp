#ifndef _MOTION_HPP_
#define _MOTION_HPP_

#include "structures.hpp"
#include "WorldModel.hpp"
#include "Radio.hpp"

class Motion
{

public:

  //Generic Constructor
  Motion();

  //Link to modules
  void setWM(WorldModel *wm);
  void setRadio(Radio *rad);

  //updates

  void updateFromSoccer(const command_set *);

private:

  //links
  const WorldModel * wm;

  //main control structure
  const robot_control_set controls;

};

#endif //_MOTION_HPP_
