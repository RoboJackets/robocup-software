#ifndef _MOTION_HPP_
#define _MOTION_HPP_

#include "structures.hpp"

class Motion
{

public:

  //Generic Constructor
  Motion();

  //Link to modules
  void setWM(WorldModel *);

  //updates
  void updateFromSoccer(const command_set *);
  void updateFromWorldModel(const state_pv *);

private:
  
  //links
  WorldModel * wm;

  //main control structure
  const robot_control_set controls;
  

}

#endif //_MOTION_HPP_
