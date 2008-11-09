#ifndef _DATA_INPUT_HPP_
#define _DATA_INPUT_HPP_

#include "structures.hpp"
#include "WorldModel.hpp"

class DataInput
{

public:

  //Generic Constructor
  DataInput();

  //Setup and start UDP Connection
  //Loops with blocking read to connection,
  //and triggers dataflow when message comes in
  void init();

  //Set World Module
  void setWM(WorldModel *);

private:

  //link to world module
  WorldModel * wm;


};

#endif //_DATA_INPUT_HPP_
