#pragma once

#include <rj_param_utils/param.hpp>

namespace trajectory_utils {
    constexpr auto kMaxIterations = "trajectory_utils";
    constexpr auto kExpectedDt = "trajectory_utils";

    DECLARE_INT64(kMaxIterations, max_iterations)
    DECLARE_NS_FLOAT64(kExpectedDt, RJ::Seconds, expected_dt)
}