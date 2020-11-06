#pragma once

#include <rj_param_utils/param.hpp>

constexpr auto kSoccerParamModule = "soccer";

DECLARE_NS_FLOAT64(kSoccerParamModule, soccer::physics, kBallDecayConstant)
