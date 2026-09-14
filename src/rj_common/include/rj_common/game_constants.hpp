#pragma once

#include <string>  // for size_t

/// all distances in meters
/// all times in seconds
/// all weights in kilograms

// Number of identifiable robots on one team
// IF U CHANGE THIS NUMBER CHECK Marking.msg
inline constexpr size_t kNumShells = 16;

// Number of playing robots on one team
inline constexpr size_t kRobotsPerTeam = 6;

inline constexpr size_t kMaxDribble = 128;
inline constexpr size_t kMaxKick = 15;

inline constexpr float kShotCalculationGranularity = 0.04f;

// Constant for ball deceleration on field
inline constexpr float kBallDecel = -0.4f;

/** constants for dot patterns */
inline constexpr float kDotsSmallOffset = 0.035;
inline constexpr float kDotsLargeOffset = 0.054772;
inline constexpr float kDotsRadius = 0.02;

/** constants for planning */
inline constexpr double kAvoidBallDistance = 0.01;

inline const std::string kTeamNameLower = "robojackets";
inline const std::string kTeamName = "RoboJackets";
