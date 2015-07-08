#pragma once

#include "OfflineLqrController.hpp"

/// An array of floats that contain the values for the lookup table.
extern const float Robot2015LqrLookupTableValues[];

/// This is the lookup table containing the entries for the 2015 bot.  It is
/// automatically generated at compile time through CMake using the values
/// specified in TunableRobotParams.cpp.
extern const LqrLookupTable<OfflineLqrController::KType>
    Robot2015LqrLookupTable;
