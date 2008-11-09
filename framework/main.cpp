/*
 * Driver Application for full framework
 *
 * Provides connections between World Modeling, Soccer, and Motion Control
 */

#include <iostream>

#include "structures.hpp"

#include "DataInput.hpp"
#include "WorldModel.hpp"
#include "Soccer.hpp"
#include "Motion.hpp"

using std::cout;
using std::endl;

int main()
{

  //Create Modules
  cout << "Creating Modules..." << endl;
  DataInput in();
  WorldModel wm();
  Soccer soc();
  Motion mot();
  cout << "Modules Created!" << endl;

  //Link modules
  in.setWM(&wm); //input drives world model
  wm.setSoc(&soc); //world model drives soccer
  wm.setMot(&mot); //world model drives motion
  soc.setMot(&mot); //soccer drives commands to motion
  mot.setWM(&wm); //motion feeds back controls to world model
  
  cout << "Modules Connected!" << endl;

  //initialize/start connection to vision/simulator
  in.init();

  
  //Finish
  return 0;
}
