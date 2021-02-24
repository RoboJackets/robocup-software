#include <gtest/gtest.h>

#include "rj_param_utils/global_param.hpp"
#include "rj_param_utils/param.hpp"

namespace rj_param_utils::Testing {

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

static const std::vector<std::string> kExampleStringVecValue = {"123", "456", "789"};
constexpr auto kExampleStringVecDescription = "abcde Test.";

constexpr auto kGlobalNode = "test_global_node";
constexpr auto kReceiverNode = "test_receiver_node";

// Root namespace
DEFINE_BOOL(kGlobalNode, bare_bool, kExampleBoolValue, kExampleBoolDescription)
DEFINE_INT64(kGlobalNode, bare_int64, kExampleIntValue, kExampleIntDescription)
DEFINE_FLOAT64(kGlobalNode, bare_double, kExampleDoubleValue, kExampleDoubleDescription)

DEFINE_BYTE_VEC(kGlobalNode, bare_byte_vec, kExampleByteVecValue, kExampleByteVecDescription)
DEFINE_BOOL_VEC(kGlobalNode, bare_bool_vec, kExampleBoolVecValue, kExampleBoolVecDescription)
DEFINE_INT64_VEC(kGlobalNode, bare_int64_vec, kExampleIntVecValue, kExampleIntVecDescription)
DEFINE_STRING_VEC(kGlobalNode, bare_string_vec, kExampleStringVecValue, kExampleStringVecDescription)

// Namespaced
DEFINE_NS_STRING(kGlobalNode, test::hello, namespaced_string, kExampleStringValue,
                 kExampleStringDescription)
DEFINE_NS_STRING(kGlobalNode, test::bye, namespaced_string, kExampleStringValue2,
                 kExampleStringDescription2)
DEFINE_NS_FLOAT64_VEC(kGlobalNode, test::byte, namespaced_double_vec, kExampleDoubleVecValue,
                      kExampleDoubleVecDescription)

class ReceiverNode : public rclcpp::Node {
public:
    ReceiverNode() : rclcpp::Node("receiver_node") {};

    ::params::ROS2GlobalParamProvider global_param_provider_ = ::params::ROS2GlobalParamProvider(this, kGlobalNode);
};

class GlobalParamProviderTest : public ::testing::Test {
public:
    void SetUp() override {
        rclcpp::init(0, {});

        global_node_ = std::make_shared<rclcpp::Node>(kGlobalNode);
        receiver_node_ = std::make_shared<ReceiverNode>();
        global_node_->declare_parameter("global_bool", true);
        global_node_->declare_parameter("global_float", 3.14f);

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        executor_->add_node(global_node_);
        executor_->add_node(receiver_node_);
    }

    void TearDown() override {
        is_running_ = false;
        ros_thread_.join();
        rclcpp::shutdown();
    }
protected:
    void start_ros() {
        is_running_ = true;
        ros_thread_ = std::thread{[this]() {
            while (is_running_ && rclcpp::ok()) {
                executor_->spin_once();
            }
        }};
    }

    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<rclcpp::Node> global_node_;
    std::shared_ptr<ReceiverNode> receiver_node_;
    std::thread ros_thread_;
    std::atomic<bool> is_running_;
};

TEST_F(GlobalParamProviderTest, RequestUpdateParam) {
    start_ros();
    // auto receiver_bool_param = receiver_node_->global_param_provider_.params_client_->get_parameters({"global_bool"});
    // auto receiver_float_param = receiver_node_->global_param_provider_.params_client_->get_parameters({"global_float"});
    // ASSERT_TRUE(receiver_bool_param.get()[1].as_bool());
    // ASSERT_EQ(receiver_float_param.get()[1].as_double(), 3.14f);
    // bool bool_value{};
    // auto receiver_bool = receiver_node_->global_param_provider_.Get("bare_bool", &bool_value);
    // double double_value{};
    // auto receiver_double = receiver_node_->global_param_provider_.Get("bare_double", &double_value);
    ASSERT_TRUE(PARAM_bare_bool);
    ASSERT_EQ(PARAM_bare_double, kExampleDoubleValue);

    // Update parameter values
    // auto new_bool = rclcpp::Parameter("bare_bool", false);
    // auto new_float = rclcpp::Parameter("bare_double", 6.28f);
    // global_node_->set_parameter(new_bool);
    // global_node_->set_parameter(new_float);
    // ASSERT_FALSE(receiver_bool_param.get()[1].as_bool());
    // ASSERT_EQ(receiver_float_param.get()[1].as_double(), 6.28f);
    // ASSERT_FALSE(PARAM_bare_bool);
    // ASSERT_EQ(PARAM_bare_double, 6.28f);
}

TEST_F(GlobalParamProviderTest, CorrectBareDefaultBareValue) {
    start_ros();
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
TEST_F(GlobalParamProviderTest, CorrectDefaultNamespaceValue) {
    start_ros();
    EXPECT_EQ(test::hello::PARAM_namespaced_string, kExampleStringValue);

    ASSERT_EQ(test::byte::PARAM_namespaced_double_vec.size(), kExampleDoubleVecValue.size());
    for (size_t i = 0; i < test::byte::PARAM_namespaced_double_vec.size(); i++) {
        EXPECT_EQ(test::byte::PARAM_namespaced_double_vec[i], kExampleDoubleVecValue[i]);
    }
}
} // namespace rj_param_utils
