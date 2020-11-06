#pragma once

#include <rj_param_utils/param.hpp>

constexpr auto soccerParamModule = "soccer";

DECLARE_NS_FLOAT64(soccerParamModule, soccer::physics, kBallDecayConstant)
