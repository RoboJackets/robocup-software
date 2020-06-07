#pragma once

#include <field_dimensions.h>

#include <string>  // for size_t

/// all distances in meters
/// all times in seconds
/// all weights in kilograms

// Number of identifiable robots on one team
constexpr size_t Num_Shells = 16;

// Number of playing robots on one team
constexpr size_t Robots_Per_Team = 6;

constexpr size_t Max_Dribble = 128;
constexpr size_t Max_Kick = 255;

constexpr float Ball_Diameter = 0.043f;
constexpr float Ball_Radius = Ball_Diameter / 2.0f;
constexpr float Ball_Mass = 0.048f;

constexpr float Robot_Diameter = 0.180f;
constexpr float Robot_Radius = Robot_Diameter / 2.0f;
constexpr float Robot_Height = 0.150f;
constexpr float Robot_MouthWidth = 0.0635f;
constexpr float Robot_MouthRadius = 0.078f;

/** constants for dot patterns */
constexpr float Dots_Small_Offset = 0.035;
constexpr float Dots_Large_Offset = 0.054772;
constexpr float Dots_Radius = 0.02;

const std::string Team_Name_Lower = "robojackets";
const std::string Team_Name = "RoboJackets";
