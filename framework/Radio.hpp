#ifndef _RADIO_HPP_
#define _RADIO_HPP_

#include "structures.hpp"

class Radio
{

public:

  //Generic Constructor
  Radio();

  //updates
  void updateFromMotion(const robot_control_set *);

  //initialize radio connection
  void init();

  //private:
  

};

#endif //_RADIO_HPP_
