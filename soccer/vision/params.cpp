#include "vision/params.hpp"

namespace vision_filter {
DEFINE_FLOAT64(kVisionFilterParamModule, vision_loop_dt, 1.0 / 60.0,
               "1/freq of the vision loop. In seconds.")

DEFINE_INT64(kVisionFilterParamModule, max_num_cameras, 12,
             "Max number of cameras possible on the field.")

DEFINE_NS_INT64(kVisionFilterParamModule, filter::health, init, 2,
                "Initial health of the Kalman filters. Must be between min and max.")
DEFINE_NS_INT64(kVisionFilterParamModule, filter::health, inc, 2,
                "How much to increment every measurement.")
DEFINE_NS_INT64(kVisionFilterParamModule, filter::health, dec, 1,
                "How much to decrement every predict without measurement.")
DEFINE_NS_INT64(kVisionFilterParamModule, filter::health, max, 20, "Max health of the filters.")
DEFINE_NS_INT64(kVisionFilterParamModule, filter::health, min, 1,
                "Min health of the filters, must be greater than 0.")

DEFINE_NS_INT64(kVisionFilterParamModule, kick::detector, slow_kick_hist_length, 5,
                "How many frames to store for the slow kick detector.")
DEFINE_NS_INT64(kVisionFilterParamModule, kick::detector, fast_kick_hist_length, 3,
                "How many frames to store for the fast kick detector.")
}  // namespace vision_filter