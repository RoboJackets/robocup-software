#include <gtest/gtest.h>
#include <rj_param_utils/param.h>

namespace params::testing {
constexpr bool kExampleBoolValue = true;
constexpr auto kExampleBoolDescription = "abcdefg";

constexpr int64_t kExampleIntValue = 1234;
constexpr auto kExampleIntDescription = "Test 123.";

constexpr double kExampleDoubleValue = 1.234;
constexpr auto kExampleDoubleDescription = "Test 456.";

static const std::string kExampleStringValue = "Test please.";
constexpr auto kExampleStringDescription = "Lets go.";

static const std::vector<uint8_t> kExampleByteVecValue = {0xB0, 0xBA, 0xCA};
constexpr auto kExampleByteVecDescription = "lmao nice.";

static const std::vector<bool> kExampleBoolVecValue = {false, false, true};
constexpr auto kExampleBoolVecDescription = "ayy lmao.";

static const std::vector<int64_t> kExampleIntVecValue = {1, 2, 3, 4, 5};
constexpr auto kExampleIntVecDescription = "1234 Test.";

static const std::vector<double> kExampleDoubleVecValue = {1.0, 2.14, 3.2, 4.1};
constexpr auto kExampleDoubleVecDescription = "1234 Test.";

static const std::vector<std::string> kExampleStringVecValue = {"123", "456",
                                                                "789"};
constexpr auto kExampleStringVecDescription = "abcde Test.";

// Root namespace
DEFINE_BOOL(bare_bool, kExampleBoolValue, kExampleBoolDescription)
DEFINE_INT64(bare_int64, kExampleIntValue, kExampleIntDescription)
DEFINE_FLOAT64(bare_double, kExampleDoubleValue, kExampleDoubleDescription)

DEFINE_BYTE_VEC(bare_byte_vec, kExampleByteVecValue, kExampleByteVecDescription)
DEFINE_BOOL_VEC(bare_bool_vec, kExampleBoolVecValue, kExampleBoolVecDescription)
DEFINE_INT64_VEC(bare_int64_vec, kExampleIntVecValue, kExampleIntVecDescription)
DEFINE_STRING_VEC(bare_string_vec, kExampleStringVecValue,
                  kExampleStringVecDescription)

// Namespaced
DEFINE_NAMESPACED_STRING(test.hello, namespaced_string, kExampleStringValue,
                         kExampleStringDescription)
DEFINE_NAMESPACED_FLOAT64_VEC(test.byte, namespaced_double_vec,
                              kExampleDoubleVecValue,
                              kExampleDoubleVecDescription)

/**
 * @brief Test that the default value of the DEFINE_* variant of defining params
 * is correct.
 */
TEST(Params, CorrectDefaultBareValue) {
    EXPECT_EQ(PARAM_bare_bool, kExampleBoolValue);
    EXPECT_EQ(PARAM_bare_int64, kExampleIntValue);
    EXPECT_EQ(PARAM_bare_double, kExampleDoubleValue);

    ASSERT_EQ(PARAM_bare_byte_vec.size(), kExampleByteVecValue.size());
    for (size_t i = 0; i < PARAM_bare_byte_vec.size(); i++) {
        EXPECT_EQ(PARAM_bare_byte_vec[i], kExampleByteVecValue[i]);
    }

    ASSERT_EQ(PARAM_bare_bool_vec.size(), kExampleBoolVecValue.size());
    for (size_t i = 0; i < PARAM_bare_bool_vec.size(); i++) {
        EXPECT_EQ(PARAM_bare_bool_vec[i], kExampleBoolVecValue[i]);
    }

    ASSERT_EQ(PARAM_bare_int64_vec.size(), kExampleIntVecValue.size());
    for (size_t i = 0; i < PARAM_bare_int64_vec.size(); i++) {
        EXPECT_EQ(PARAM_bare_int64_vec[i], kExampleIntVecValue[i]);
    }

    ASSERT_EQ(PARAM_bare_string_vec.size(), kExampleStringVecValue.size());
    for (size_t i = 0; i < PARAM_bare_string_vec.size(); i++) {
        EXPECT_EQ(PARAM_bare_string_vec[i], kExampleStringVecValue[i]);
    }
}

/**
 * @brief Test that the default value of the DEFINE_NAMESPACED_* variant of
 * defining params is correct.
 */
TEST(Params, CorrectDefaultNamespaceValue) {
    EXPECT_EQ(PARAM_namespaced_string, kExampleStringValue);

    ASSERT_EQ(PARAM_namespaced_double_vec.size(),
              kExampleDoubleVecValue.size());
    for (size_t i = 0; i < PARAM_namespaced_double_vec.size(); i++) {
        EXPECT_EQ(PARAM_namespaced_double_vec[i], kExampleDoubleVecValue[i]);
    }
}

/**
 * @brief Tests ParamProvider::Update for parameters that are defined using
 * DEFINE_*.
 */
TEST(Params, ParamProviderGet) {
    ::params::ParamProvider provider;

    bool bool_value{};
    EXPECT_TRUE(provider.Get("bare_bool", &bool_value));
    EXPECT_EQ(PARAM_bare_bool, bool_value);

    int64_t int_value{};
    ASSERT_TRUE(provider.Get("bare_int64", &int_value));
    EXPECT_EQ(PARAM_bare_int64, int_value);

    double double_value{};
    ASSERT_TRUE(provider.Get("bare_double", &double_value));
    EXPECT_EQ(PARAM_bare_double, double_value);

    double fake_double{};
    EXPECT_FALSE(provider.Get("fake_double", &fake_double));

    std::string fake_string{};
    EXPECT_FALSE(provider.Get("fake_string", &fake_string));

    int64_t fake_int_vec{};
    EXPECT_FALSE(provider.Get("bare_int64_vec", &fake_int_vec));

    std::string fake_double_vec{};
    EXPECT_FALSE(
        provider.Get("test.byte.namespaced_double_vec", &fake_double_vec));
}

/**
 * @brief Tests ParamProvider::Update for parameters that are defined using
 * DEFINE_*.
 */
TEST(Params, ParamProviderUpdate) {
    ::params::ParamProvider provider;

    constexpr bool kNewBoolValue = false;
    EXPECT_EQ(PARAM_bare_bool, kExampleBoolValue);
    provider.Update("bare_bool", kNewBoolValue);
    EXPECT_EQ(PARAM_bare_bool, kNewBoolValue);

    constexpr int64_t kNewInt64Value = 837122;
    EXPECT_EQ(PARAM_bare_int64, kExampleIntValue);
    provider.Update("bare_int64", kNewInt64Value);
    EXPECT_EQ(PARAM_bare_int64, kNewInt64Value);

    constexpr double kNewDoubleValue = 0.321091;
    EXPECT_EQ(PARAM_bare_double, kExampleDoubleValue);
    provider.Update("bare_double", kNewDoubleValue);
    EXPECT_EQ(PARAM_bare_double, kNewDoubleValue);
}

/**
 * @brief Tests ParamProvider::Update for parameters that are defined using
 * DEFINE_NAMESPACED_*.
 */
TEST(Params, ParamProviderUpdateNamespace) {
    ::params::ParamProvider provider;

    const std::string kNewString = "Haha.";
    EXPECT_EQ(PARAM_namespaced_string, kExampleStringValue);
    provider.Update("test.hello.namespaced_string", kNewString);
    EXPECT_EQ(PARAM_namespaced_string, kNewString);

    const std::vector<double> kNewDoubleVec{0.3, 0.4, 0.1, 0.5};
    EXPECT_EQ(PARAM_namespaced_double_vec, kExampleDoubleVecValue);
    provider.Update("test.byte.namespaced_double_vec", kNewDoubleVec);
    EXPECT_EQ(PARAM_namespaced_double_vec, kNewDoubleVec);
}
}  // namespace params::testing
