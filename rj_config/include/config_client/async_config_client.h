#include <config_client/config_client_node.h>

#include <thread>

namespace config_client {
/**
 * @brief Wrapper over ConfigClientNode that creates a thread that does all the
 * ROS2 spinning.
 */
class AsyncConfigClient {
public:
    using UniquePtr = std::unique_ptr<AsyncConfigClient>;

    /**
     * @brief Constructs an AsyncConfigClient.
     * @param name The name of the ConfigClientNode instance.
     */
    AsyncConfigClient(const std::string& name);

    /**
     * @brief Returns the ConfigClientNode instance.
     * @return the ConfigClientInstance node.
     */
    ConfigClientNode& config_client_node();

private:
    std::shared_ptr<ConfigClientNode> config_client_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_thread_;
};
}  // namespace config_client