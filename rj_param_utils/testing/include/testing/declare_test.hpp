#include <rj_param_utils/param.hpp>

namespace params::testing {
constexpr auto kDeclareTestModule = "declare_test";

constexpr int64_t kDeclareIntValue = 1888;
constexpr auto kDeclareIntDescription = "Yay.";

constexpr double kDeclareDoubleValue = 1.321154;
constexpr auto kDeclareDoubleDescription = "Nope.";

DECLARE_INT64(kDeclareTestModule, bare_declare_int)
DECLARE_NS_FLOAT64(kDeclareTestModule, a::b, declare_double)
}  // namespace params::testing
