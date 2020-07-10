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
static const std::string kExampleStringValue2 = "Goodbye.";
constexpr auto kExampleStringDescription2 = "Nice description.";
static const std::string kExampleStringValue3 = "See you.";
constexpr auto kExampleStringDescription3 = "Hello there.";

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

constexpr auto kModule = "test_module";
constexpr auto kModule2 = "test_module2";

// Root namespace
DEFINE_BOOL(kModule, bare_bool, kExampleBoolValue, kExampleBoolDescription)
DEFINE_INT64(kModule, bare_int64, kExampleIntValue, kExampleIntDescription)
DEFINE_FLOAT64(kModule, bare_double, kExampleDoubleValue,
               kExampleDoubleDescription)

DEFINE_BYTE_VEC(kModule, bare_byte_vec, kExampleByteVecValue,
                kExampleByteVecDescription)
DEFINE_BOOL_VEC(kModule, bare_bool_vec, kExampleBoolVecValue,
                kExampleBoolVecDescription)
DEFINE_INT64_VEC(kModule, bare_int64_vec, kExampleIntVecValue,
                 kExampleIntVecDescription)
DEFINE_STRING_VEC(kModule, bare_string_vec, kExampleStringVecValue,
                  kExampleStringVecDescription)

// Namespaced
DEFINE_NS_STRING(kModule, test::hello, namespaced_string, kExampleStringValue,
                 kExampleStringDescription)
DEFINE_NS_STRING(kModule, test::bye, namespaced_string, kExampleStringValue2,
                 kExampleStringDescription2)
DEFINE_NS_FLOAT64_VEC(kModule, test::byte, namespaced_double_vec,
                      kExampleDoubleVecValue, kExampleDoubleVecDescription)

// In a different "module", emulating multiple nodes in a single
// executable.
DEFINE_NS_STRING(kModule2, test::hello, different_module, kExampleStringValue3,
                 kExampleStringDescription3)

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
 * @brief Test that the default value of the DEFINE_NS_* variant of
 * defining params is correct.
 */
TEST(Params, CorrectDefaultNamespaceValue) {
    EXPECT_EQ(test::hello::PARAM_namespaced_string, kExampleStringValue);

    ASSERT_EQ(test::byte::PARAM_namespaced_double_vec.size(),
              kExampleDoubleVecValue.size());
    for (size_t i = 0; i < test::byte::PARAM_namespaced_double_vec.size();
         i++) {
        EXPECT_EQ(test::byte::PARAM_namespaced_double_vec[i],
                  kExampleDoubleVecValue[i]);
    }
}

/**
 * @brief Checks that the param metadata, ie. fullname, description are correct.
 */
TEST(Params, CorrectMetadata) {
    ::params::ParamProvider provider{kModule};
    {
        const auto& param_map = provider.GetParamMap<bool>();
        const auto it = param_map.find("bare_bool");
        ASSERT_TRUE(it != param_map.end());
        EXPECT_EQ(it->second->prefix(), "");
        EXPECT_EQ(it->second->name(), "bare_bool");
        EXPECT_EQ(it->second->full_name(), "bare_bool");
        EXPECT_EQ(it->second->help(), kExampleBoolDescription);
    }

    {
        const auto& param_map = provider.GetParamMap<int64_t>();
        const auto it = param_map.find("bare_int64");
        ASSERT_NE(it, param_map.end());
        EXPECT_EQ(it->second->prefix(), "");
        EXPECT_EQ(it->second->name(), "bare_int64");
        EXPECT_EQ(it->second->full_name(), "bare_int64");
        EXPECT_EQ(it->second->help(), kExampleIntDescription);
    }

    {
        const auto& param_map = provider.GetParamMap<std::vector<double>>();
        const auto it = param_map.find("test::byte::namespaced_double_vec");
        ASSERT_NE(it, param_map.end());
        EXPECT_EQ(it->second->prefix(), "test::byte");
        EXPECT_EQ(it->second->name(), "namespaced_double_vec");
        EXPECT_EQ(it->second->full_name(), "test::byte::namespaced_double_vec");
        EXPECT_EQ(it->second->help(), kExampleDoubleVecDescription);
    }
}

/**
 * @brief Tests ParamProvider::Update for parameters that are defined using
 * DEFINE_*.
 */
TEST(Params, ParamProviderGet) {
    ::params::ParamProvider provider{kModule};

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
    ::params::ParamProvider provider{kModule};

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
 * DEFINE_NS_*.
 */
TEST(Params, ParamProviderUpdateNamespace) {
    ::params::ParamProvider provider{kModule};

    const std::string kNewString = "Haha.";
    EXPECT_EQ(test::hello::PARAM_namespaced_string, kExampleStringValue);
    provider.Update("test::hello::namespaced_string", kNewString);
    EXPECT_EQ(test::hello::PARAM_namespaced_string, kNewString);

    const std::string kNewString2 = "Jokes on you.";
    EXPECT_EQ(test::bye::PARAM_namespaced_string, kExampleStringValue2);
    provider.Update("test::bye::namespaced_string", kNewString2);
    EXPECT_EQ(test::bye::PARAM_namespaced_string, kNewString2);

    const std::vector<double> kNewDoubleVec{0.3, 0.4, 0.1, 0.5};
    EXPECT_EQ(test::byte::PARAM_namespaced_double_vec, kExampleDoubleVecValue);
    provider.Update("test::byte::namespaced_double_vec", kNewDoubleVec);
    EXPECT_EQ(test::byte::PARAM_namespaced_double_vec, kNewDoubleVec);
}

/**
 * @brief Tests that ParamProvider only picks up parameters from its own module.
 */
TEST(Params, ParamProviderModules) {
    ::params::ParamProvider provider{kModule};
    ASSERT_TRUE(
        provider.HasParam<std::string>("test::hello::namespaced_string"));
    ASSERT_FALSE(
        provider.HasParam<std::string>("test::hello::different_module"));

    ::params::ParamProvider provider2{kModule2};
    ASSERT_FALSE(
        provider2.HasParam<std::string>("test::hello::namespaced_string"));
    ASSERT_TRUE(
        provider2.HasParam<std::string>("test::hello::different_module"));
}
}  // namespace params::testing
