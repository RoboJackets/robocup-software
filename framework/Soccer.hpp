#ifndef _SOCCER_HPP_
#define _SOCCER_HPP_

#include "structures.hpp"
#include "Motion.hpp"

class Soccer
{

public:

  //Generic Constructor
  Soccer();

  //Link to modules
  void setMot(Motion *);

  //Update from world model
  void update(const state_pv *);

private:
  
  //links
  Motion * mot;

}

#endif //_SOCCER_HPP_
