#include <rj_param_utils/param.hpp>

namespace trajectory_utils {
    DEFINE_INT64(kMaxIterations, max_iterations, 100, "Limit on iterations")
    DEFINE_NS_FLOAT64(kExpectedDt, RJ::Seconds, expected_dt, 0.05, "Delta time between program is started and first message received")
}