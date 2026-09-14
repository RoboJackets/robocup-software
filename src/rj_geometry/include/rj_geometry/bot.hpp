#pragma once

// Geometry derived from RoboCup Rules
// https://robocup-ssl.github.io/ssl-rules/sslrules.pdf

inline constexpr float kRobotDiameter = 0.18f; // m
inline constexpr float kRobotRadius = 0.09f; // m
inline constexpr float kRobotHeight = 0.15f; // m
// We use a mouth cut smaller than the max permitted by the rules.
inline constexpr float kRobotMouthWidth = 0.0635f; // m
inline constexpr float kRobotMouthRadius = 0.078f; // m
