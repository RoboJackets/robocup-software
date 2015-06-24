#pragma once

#include "OfflineLqrController.hpp"
#include "RobotModel.hpp"

/// This global variable contains the values for our 2015 robot.  It is declared
/// here and defined in TunableRobotParameters.cpp - go there to edit the
/// values.
extern const RobotModelParams Robot2015ModelParams;

// This global variable contains the RobotModel that is built from the
// Robot2015ModelParams.
extern const RobotModel Robot2015SystemModel;

/// The weights for the 2015 bot are declared here and defined in
/// TunableRobotParameters.cpp - go there to make changes.
extern const LqrCostFunctionWeights Robot2015LqrCostFunctionWeights;

/// These are the weighting matrices for LQR, constructed from
/// Robot2015LqrCostFunctionWeights.
extern const OfflineLqrController::QType Robot2015LqrQ;
extern const OfflineLqrController::RType Robot2015LqrR;

/// Min and max rotational velocities we're generating a lookup table for
extern const float Robot2015LqrLookupTableMinRotVel;
extern const float Robot2015LqrLookupTableMaxRotVel;

/// Number of lookup table entries to generate
extern const size_t Robot2015LqrLookupTableNumEntries;
