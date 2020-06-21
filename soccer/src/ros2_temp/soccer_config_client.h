#include <config_client/config_client_node.h>

#include <Context.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_temp {

/**
 * @brief Temporary class for allowing Processor to access the configs in
 * ConfigClient. Will be removed when everything is properly migrated to ROS2.
 */
class SoccerConfigClient {
public:
    SoccerConfigClient(Context* context);

    /**
     * @brief Updates config_server with the current values of GameSettings and
     * FieldDimensions, spins config_client_, then sets context with the values
     * of FieldDimensions in context.
     */
    void run();

private:
    Context* context_;
    std::shared_ptr<config_client::ConfigClientNode> config_client_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;

    /**
     * @brief Calls executor_.spin().
     */
    void spin();
};
}  // namespace ros2_temp
