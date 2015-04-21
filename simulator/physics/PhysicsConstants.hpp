#pragma once

#include <Constants.hpp>

//Bullet was designed for dimensions and velocities in the range (0.05 to 10).
//By default, Bullet assumes units to be in meters and time in seconds

//Conversion factor
const float scaling = 10.f; //

//Physics constants
const float Gravity = -9.81f*scaling;

//Measurements
const float Sim_Ball_Diameter = Ball_Diameter*scaling;
const float Sim_Ball_Radius = Ball_Radius*scaling;
const float Sim_Ball_Mass = Ball_Mass; //mass is not scaled

/** Dimensions of the goal walls **/
const float Sim_GoalWall_Width = 0.02*scaling;
const float Sim_GoalWall_Height = 0.16*scaling;

const float Sim_Robot_Diameter = Robot_Diameter*scaling;
const float Sim_Robot_Radius = Robot_Radius*scaling;
const float Sim_Robot_Height = Robot_Height*scaling;
const float Sim_Robot_MouthWidth = Robot_MouthWidth*scaling;
const float Sim_Robot_Mass = 10; //mass is not scaled

//FIXME: These are guessed
const float Sim_Wheel_Diameter = 0.05f*scaling;
const float Sim_Wheel_Radius = Sim_Wheel_Diameter/2.f;
const float Sim_Wheel_Width = 0.02f*scaling;
