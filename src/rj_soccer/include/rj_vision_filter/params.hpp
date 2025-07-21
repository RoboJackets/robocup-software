#pragma once

#include <rj_param_utils/param.hpp>

namespace vision_filter {
constexpr auto kVisionFilterParamModule = "vision_filter";

DECLARE_FLOAT64(kVisionFilterParamModule, vision_loop_dt)

DECLARE_INT64(kVisionFilterParamModule, max_num_cameras)

DECLARE_NS_INT64(kVisionFilterParamModule, filter::health, init)
DECLARE_NS_INT64(kVisionFilterParamModule, filter::health, inc)
DECLARE_NS_INT64(kVisionFilterParamModule, filter::health, dec)
DECLARE_NS_INT64(kVisionFilterParamModule, filter::health, max)
DECLARE_NS_INT64(kVisionFilterParamModule, filter::health, min)

DECLARE_NS_INT64(kVisionFilterParamModule, kick::detector,
                 slow_kick_hist_length)
DECLARE_NS_INT64(kVisionFilterParamModule, kick::detector,
                 fast_kick_hist_length)
}  // namespace vision_filter