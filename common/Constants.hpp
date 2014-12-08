
#pragma once

#include <string> // for size_t

#include "Field_Dimensions_t.hpp"

/// all distances in meters
/// all times in seconds
/// all weights in kilograms

// Number of identifiable robots on one team
const size_t Num_Shells = 16;

// Number of playing robots on one team
const size_t Robots_Per_Team = 6;

const float Ball_Diameter = 0.043f;
const float Ball_Radius = Ball_Diameter/2.0f;
const float Ball_Mass = 0.048f;

/** Single-size field constants */
extern const Field_Dimensions_t Single_Field_Dimensions;

/** Single-size field constants */
extern const Field_Dimensions_t Double_Field_Dimensions;

/** Current field constants */
extern Field_Dimensions_t Field_Dimensions;

const float Field_Length = 6.05f;
const float Field_Width = 4.05f;
const float Field_Border = 0.25f;

const float Field_LineWidth = 0.01f;

const float Field_GoalWidth = 0.700f;
const float Field_GoalDepth = 0.180f;
const float Field_GoalHeight = 0.160f;

/** Distance of the penalty marker from the goal line */
const float Field_PenaltyDist = 0.750f;
const float Field_PenaltyDiam = 0.010f;

/** Radius of the goal arcs */
const float Field_ArcRadius = 0.8f;

/** diameter of the center circle */
const float Field_CenterRadius = 0.5f;
const float Field_CenterDiameter = Field_CenterRadius * 2.0f;

/** flat area for defence markings */
const float Field_GoalFlat = 0.35f;

const float Floor_Length = Field_Length + 2.0 * Field_Border;
const float Floor_Width = Field_Width + 2.0 * Field_Border;

const float Robot_Diameter = 0.180f;
const float Robot_Radius = Robot_Diameter/2.0f;
const float Robot_Height = 0.150f;
const float Robot_MouthWidth = 0.10f;

/** constants for dot patterns */
const float Dots_Small_Offset = 0.035;
const float Dots_Large_Offset = 0.054772;
const float Dots_Radius = 0.02;

/// timestamp() returns the current time in microseconds.  Multiply by this constant to get to seconds.
const float TimestampToSecs = 1.0f / 1000000.0f;
const float SecsToTimestamp = 1.0f / TimestampToSecs;
