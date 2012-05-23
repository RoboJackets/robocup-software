#pragma once

#include <Constants.hpp>

//Bullet was designed for dimensions and velocities in the range (0.05 to 10).
//By default, Bullet assumes units to be in meters and time in seconds
//We scale to decimeter (m^-1) for more realistic simulations

//Conversion factor
const float mtodm = 10.f;

//Physics constants
const float Gravity = -9.81f*mtodm;

//Measurements
const float Sim_Ball_Diameter = Ball_Diameter*mtodm;
const float Sim_Ball_Radius = Ball_Radius*mtodm;
const float Sim_Ball_Mass = Ball_Mass; //mass is not scaled

const float Sim_Field_Length = Field_Length*mtodm;
const float Sim_Field_Width = Field_Width*mtodm;
const float Sim_Field_Border = Field_Border*mtodm;

const float Sim_Field_LineWidth = Field_LineWidth*mtodm;

const float Sim_Field_GoalWidth = Field_GoalWidth*mtodm;
const float Sim_Field_GoalDepth = Field_GoalDepth*mtodm;
const float Sim_Field_GoalHeight = Field_GoalHeight*mtodm;

/** Distance of the penalty marker from the goal line */
const float Sim_Field_PenaltyDist = Field_PenaltyDist*mtodm;
const float Sim_Field_PenaltyDiam = Field_PenaltyDiam*mtodm;

/** Radius of the goal arcs */
const float Sim_Field_ArcRadius = Field_ArcRadius*mtodm;

/** diameter of the center circle */
const float Sim_Field_CenterRadius = Field_CenterRadius*mtodm;
const float Sim_Field_CenterDiameter = Field_CenterDiameter*mtodm;

/** flat area for defence markings */
const float Sim_Field_GoalFlat = Field_GoalFlat*mtodm;

const float Sim_Floor_Length = Floor_Length*mtodm;
const float Sim_Floor_Width = Floor_Width*mtodm;

const float Sim_Robot_Diameter = Robot_Diameter*mtodm;
const float Sim_Robot_Radius = Robot_Radius*mtodm;
const float Sim_Robot_Height = Robot_Height*mtodm;
const float Sim_Robot_MouthWidth = Robot_MouthWidth*mtodm;
const float Sim_Robot_Mass = 10; //mass is not scaled

//FIXME: These are guessed
const float Sim_Wheel_Diameter = 0.1f*mtodm;
const float Sim_Wheel_Radius = Sim_Wheel_Diameter/2.f;
const float Sim_Wheel_Width = 0.4f*mtodm;
