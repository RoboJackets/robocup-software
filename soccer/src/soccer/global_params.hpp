#pragma once

#include <rj_param_utils/param.hpp>

constexpr auto kGlobalParamModule = "soccer";

DECLARE_NS_FLOAT64(kGlobalParamModule, soccer::physics, ball_decay_constant)
