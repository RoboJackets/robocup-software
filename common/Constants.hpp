
#pragma once

#include <string>  // for size_t

#include "Field_Dimensions.hpp"

/// all distances in meters
/// all times in seconds
/// all weights in kilograms

// Number of identifiable robots on one team
const size_t Num_Shells = 16;

// Number of playing robots on one team
const size_t Robots_Per_Team = 6;

const float Ball_Diameter = 0.043f;
const float Ball_Radius = Ball_Diameter / 2.0f;
const float Ball_Mass = 0.048f;

const float Robot_Diameter = 0.180f;
const float Robot_Radius = Robot_Diameter / 2.0f;
const float Robot_Height = 0.150f;
const float Robot_MouthWidth = 0.10f;

/** constants for dot patterns */
const float Dots_Small_Offset = 0.035;
const float Dots_Large_Offset = 0.054772;
const float Dots_Radius = 0.02;

/// timestamp() returns the current time in microseconds.  Multiply by this
/// constant to get to seconds.
const float SecsToTimestamp = 1000000.0f;
const float TimestampToSecs = 1.0f / SecsToTimestamp;
