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

const float Sim_Field_Length = Field_Dimensions.Length*scaling;
const float Sim_Field_Width = Field_Dimensions.Width*scaling;
const float Sim_Field_Border = Field_Dimensions.Border*scaling;

const float Sim_Field_LineWidth = Field_Dimensions.LineWidth*scaling;

const float Sim_Field_GoalWidth = Field_Dimensions.GoalWidth*scaling;
const float Sim_Field_GoalDepth = Field_Dimensions.GoalDepth*scaling;
const float Sim_Field_GoalHeight = Field_Dimensions.GoalHeight*scaling;

/** Dimensions of the goal walls **/
const float Sim_GoalWall_Width = 0.02*scaling;
const float Sim_GoalWall_Height = 0.16*scaling;

/** Distance of the penalty marker from the goal line */
const float Sim_Field_PenaltyDist = Field_Dimensions.PenaltyDist*scaling;
const float Sim_Field_PenaltyDiam = Field_Dimensions.PenaltyDiam*scaling;

/** Radius of the goal arcs */
const float Sim_Field_ArcRadius = Field_Dimensions.ArcRadius*scaling;

/** diameter of the center circle */
const float Sim_Field_CenterRadius = Field_Dimensions.CenterRadius*scaling;
const float Sim_Field_CenterDiameter = Field_Dimensions.CenterDiameter*scaling;

/** flat area for defence markings */
const float Sim_Field_GoalFlat = Field_Dimensions.GoalFlat*scaling;

const float Sim_RefArea_Offset = 0.425*scaling;
const float Sim_Floor_Length = Field_Dimensions.FloorLength*scaling + Sim_RefArea_Offset;
const float Sim_Floor_Width = Field_Dimensions.FloorWidth*scaling + Sim_RefArea_Offset;

const float Sim_Robot_Diameter = Robot_Diameter*scaling;
const float Sim_Robot_Radius = Robot_Radius*scaling;
const float Sim_Robot_Height = Robot_Height*scaling;
const float Sim_Robot_MouthWidth = Robot_MouthWidth*scaling;
const float Sim_Robot_Mass = 10; //mass is not scaled

//FIXME: These are guessed
const float Sim_Wheel_Diameter = 0.05f*scaling;
const float Sim_Wheel_Radius = Sim_Wheel_Diameter/2.f;
const float Sim_Wheel_Width = 0.02f*scaling;
