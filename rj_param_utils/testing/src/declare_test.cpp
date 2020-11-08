#include <testing/declare_test.hpp>

namespace params::testing {
DEFINE_INT64("declare_test", bare_declare_int, kDeclareIntValue, kDeclareIntDescription)
DEFINE_NS_FLOAT64(kDeclareTestModule, a::b, declare_double, kDeclareDoubleValue,
                  kDeclareDoubleDescription)
}  // namespace params::testing
