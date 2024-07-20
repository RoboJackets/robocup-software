#pragma once

#include <string>  // for size_t

/// all distances in meters
/// all times in seconds
/// all weights in kilograms

// Number of identifiable robots on one team
constexpr size_t kNumShells = 16;

// Number of playing robots on one team
constexpr size_t kRobotsPerTeam = 6;

constexpr size_t kMaxDribble = 128;
constexpr size_t kMaxKick = 255;

constexpr float kBallDiameter = 0.043f;
constexpr float kBallRadius = kBallDiameter / 2.0f;
constexpr float kBallMass = 0.048f;

constexpr float kRobotDiameter = 0.180f;
constexpr float kRobotRadius = kRobotDiameter / 2.0f;
constexpr float kRobotHeight = 0.150f;
constexpr float kRobotMouthWidth = 0.0635f;
constexpr float kRobotMouthRadius = 0.078f;

// Constant for ball deceleration on field
constexpr float kBallDecel{-0.4f};

/** constants for dot patterns */
constexpr float kDotsSmallOffset = 0.035;
constexpr float kDotsLargeOffset = 0.054772;
constexpr float kDotsRadius = 0.02;

/** constants for planning */
constexpr double kAvoidBallDistance = 0.1;

const std::string kTeamNameLower = "robojackets";
const std::string kTeamName = "RoboJackets";
