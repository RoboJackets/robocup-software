/*
 * Driver Application for full framework
 *
 * Provides connections between World Modeling, Soccer, and Motion Control
 */

#include <iostream>

using namespace std;

#include "structures.hpp"

#include "DataInput.hpp"
#include "WorldModel.hpp"
#include "Soccer.hpp"
#include "Motion.hpp"
#include "Radio.hpp"

int main()
{

  //Create Modules
  cout << "Creating Modules..." << endl;
  DataInput in();
  WorldModel wm();
  Soccer soc();
  Motion mot();
  Radio radio();
  cout << "Modules Created!" << endl;

  //Link modules
  in.setWM(&wm); //input drives world model
  wm.setSoc(&soc); //world model drives soccer
  soc.setMot(&mot); //soccer drives commands to motion
  mot.setWM(&wm); //motion feeds back controls to world model
  mot.setRadio(&radio); //send controls to radio
  
  cout << "Modules Connected!" << endl;

  //initialize/start connection to vision/simulator
  radio.init();
  in.init();

  
  //Finish
  return 0;
}
